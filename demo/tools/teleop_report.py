#!/usr/bin/env python3
"""Summarise the rc_ctrl black box CSV written by rc_ctrl_node.

Pure standard library, no ROS, no robot: it only reads the files, so it can be
run on a log directory from an earlier session without rebuilding anything.

    python3 teleop_report.py demo/log/teleop
    python3 teleop_report.py demo/log/teleop/right_1761....csv

It prints wall clock times (the file name carries the run start in epoch
milliseconds, so the monotonic column can be mapped back onto the clock):
  1. controller alarms
  2. servo send interval statistics, including missed and overlapped windows
  3. windows where the arm lagged the commanded point
  4. windows where commands were sent but the arm did not move
  5. joint rate spikes, per joint
"""

import datetime
import glob
import os
import sys

COLUMNS = [
    "t", "kind", "reason", "incr_x", "incr_y", "incr_z", "off_x", "off_y",
    "off_z", "cmd_x", "cmd_y", "cmd_z", "act_x", "act_y", "act_z", "fb_valid",
    "j1", "j2", "j3", "j4", "j5", "j6", "errcode", "ret", "held", "fb_us",
]

LAG_MM = 5.0          # |commanded - actual| that counts as lagging
LAG_MIN_S = 0.10      # only report lag windows at least this long
STALL_MM = 0.15       # mean |increment| per cycle that counts as "commanded"
STALL_MOVE_MM = 1.0   # actual movement below this is "not moving"
STALL_WIN_S = 0.20    # stall window length


def parse_start(path):
    """right_1761888123456.csv -> 1761888123.456, or None."""
    base = os.path.basename(path)
    stem = base[:-4] if base.endswith(".csv") else base
    tail = stem.rsplit("_", 1)[-1]
    try:
        return int(tail) / 1000.0
    except ValueError:
        return None


def clock(epoch):
    return datetime.datetime.fromtimestamp(epoch).strftime("%H:%M:%S")


def load(path):
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        header = handle.readline().strip().split(",")
        rows = [line.rstrip("\r\n").split(",") for line in handle if line.strip()]
    index = {name: i for i, name in enumerate(header)}
    text = ("kind", "reason")
    out = []
    for raw in rows:
        if len(raw) < len(header):
            continue
        row = {}
        try:
            for name in COLUMNS:
                if name not in index:
                    continue
                value = raw[index[name]]
                row[name] = value if name in text else float(value)
        except (ValueError, IndexError):
            continue
        out.append(row)
    return out


def windows(times, mask):
    """Merge consecutive True samples into [start, end, count, peak] windows."""
    result = []
    start = None
    peak = 0.0
    for i, flag in enumerate(mask):
        if flag:
            if start is None:
                start = i
                peak = 0.0
            peak = max(peak, times[i][1])
        elif start is not None:
            result.append((times[start][0], times[i - 1][0], i - start, peak))
            start = None
    if start is not None:
        result.append((times[start][0], times[-1][0], len(mask) - start, peak))
    return result


def norm3(a, b, c):
    return (a * a + b * b + c * c) ** 0.5


def analyse(path):
    rows = load(path)
    if not rows:
        return "  (no rows: the black box was off, or the arm never sent a servo command)"

    start = parse_start(path)
    servo = [r for r in rows if r["kind"] in ("servo",)]
    events = [r for r in rows if r["kind"] == "event"]
    t0 = servo[0]["t"] if servo else rows[0]["t"]

    def wall(t):
        if start is None:
            return "%8.2fs" % (t - t0)
        return clock(start + (t - t0))

    lines = []
    lines.append("  rows=%d  servo=%d  events=%d  span=%.1fs  start=%s"
                 % (len(rows), len(servo), len(events),
                    (rows[-1]["t"] - t0) if rows else 0.0,
                    clock(start) if start else "unknown (file name has no epoch)"))

    # 1. alarms -------------------------------------------------------------
    alarms = [r for r in events if r["reason"] == "alarm"]
    if alarms:
        for r in alarms:
            lines.append("  ALARM   %s  errcode=%d" % (wall(r["t"]), int(r["errcode"])))
    else:
        lines.append("  ALARM   none")

    # 2. send interval, only while actually moving fast ---------------------
    # A stationary hand makes the deadband skip commands for tens of
    # milliseconds, so idle gaps say nothing about the interpolation window.
    fast = []
    for i in range(1, len(servo)):
        a, b = servo[i - 1], servo[i]
        ca = norm3(a["incr_x"], a["incr_y"], a["incr_z"])
        cb = norm3(b["incr_x"], b["incr_y"], b["incr_z"])
        if ca > 0.3 and cb > 0.3:
            fast.append(b["t"] - a["t"])
    if len(fast) > 2:
        fast.sort()
        n = len(fast)
        missed = sum(1 for d in fast if d > 0.0115)
        overlap = sum(1 for d in fast if d < 0.0075)
        lines.append(
            "  SEND    moving intervals=%d median=%.2fms p99=%.2fms max=%.0fms | "
            "missed(>11.5ms)=%d overlap(<7.5ms)=%d"
            % (n, fast[n // 2] * 1000.0, fast[min(n - 1, int(n * 0.99))] * 1000.0,
               fast[-1] * 1000.0, missed, overlap))
    else:
        lines.append("  SEND    too little fast motion to judge the cadence")

    # 3. tracking lag -------------------------------------------------------
    lag_series = []
    for r in servo:
        if r.get("fb_valid", 0.0) < 0.5:
            lag_series.append((r["t"], 0.0, r))
            continue
        err = norm3(r["cmd_x"] - r["act_x"], r["cmd_y"] - r["act_y"],
                    r["cmd_z"] - r["act_z"])
        lag_series.append((r["t"], err, r))
    lag = windows([(t, e) for t, e, _ in lag_series], [e > LAG_MM for _, e, _ in lag_series])
    lag = [w for w in lag if w[1] - w[0] >= LAG_MIN_S]
    lag.sort(key=lambda w: -w[3])
    if lag:
        lines.append("  LAG     %d window(s) over %.0f mm and %.0f ms"
                     % (len(lag), LAG_MM, LAG_MIN_S * 1000))
        for start_t, end_t, count, peak in lag[:5]:
            sample = next(r for t, _, r in lag_series if t >= start_t)
            j5 = sample["j5"] * 180.0 / 3.141592653589793
            lines.append("          %s..%s  %.0fms  peak %.1f mm  j5=%.0f deg"
                         % (wall(start_t), wall(end_t), (end_t - start_t) * 1000.0,
                            peak, j5))
    else:
        lines.append("  LAG     none over %.0f mm" % LAG_MM)

    # 4. commanded but not moving -------------------------------------------
    stalls = []
    step = max(1, int(STALL_WIN_S / 0.008))
    for i in range(0, max(0, len(servo) - step), step):
        block = servo[i:i + step]
        if len(block) < step:
            break
        if any(r.get("fb_valid", 0.0) < 0.5 for r in block):
            continue
        commanded = sum(norm3(r["incr_x"], r["incr_y"], r["incr_z"]) for r in block) / len(block)
        moved = norm3(block[-1]["act_x"] - block[0]["act_x"],
                      block[-1]["act_y"] - block[0]["act_y"],
                      block[-1]["act_z"] - block[0]["act_z"])
        if commanded > STALL_MM and moved < STALL_MOVE_MM:
            stalls.append((block[0]["t"], block[-1]["t"], commanded, moved))
    if stalls:
        merged = []
        for item in stalls:
            if merged and item[0] - merged[-1][1] < 0.05:
                merged[-1] = (merged[-1][0], item[1], max(merged[-1][2], item[2]),
                              max(merged[-1][3], item[3]))
            else:
                merged.append(item)
        merged = [m for m in merged if m[1] - m[0] >= STALL_WIN_S * 0.9]
        lines.append("  STALL   %d window(s): commanded but barely moving" % len(merged))
        for start_t, end_t, commanded, moved in merged[:5]:
            sample = next(r for r in servo if r["t"] >= start_t)
            lines.append("          %s..%s  %.0fms  cmd %.2f mm/cycle, moved %.1f mm  "
                         "j3=%.0f j5=%.0f deg"
                         % (wall(start_t), wall(end_t), (end_t - start_t) * 1000.0,
                            commanded, moved,
                            sample["j3"] * 180.0 / 3.141592653589793,
                            sample["j5"] * 180.0 / 3.141592653589793))
    else:
        lines.append("  STALL   none")

    # 5. joint rate ---------------------------------------------------------
    # The joint columns only change when a feedback sample lands, which can be
    # far slower than the servo rate, so divide by the time since that joint
    # last changed instead of by the servo interval.
    names = ["j1", "j2", "j3", "j4", "j5", "j6"]
    worst = {name: (0.0, 0.0) for name in names}
    last_time = {name: servo[0]["t"] for name in names} if servo else {}
    last_value = {name: servo[0][name] for name in names} if servo else {}
    change_interval = []
    for i in range(1, len(servo)):
        t = servo[i]["t"]
        for name in names:
            value = servo[i][name]
            if value == last_value[name]:
                continue
            span = t - last_time[name]
            if span > 1e-6:
                rate = abs(value - last_value[name]) / span * 180.0 / 3.141592653589793
                if rate > worst[name][0]:
                    worst[name] = (rate, t)
                if name == "j1":
                    change_interval.append(span)
            last_value[name] = value
            last_time[name] = t
    if change_interval:
        change_interval.sort()
        lines.append("  FB      joint feedback updates every %.0f ms (median)"
                     % (change_interval[len(change_interval) // 2] * 1000.0))
    if servo:
        parts = []
        for name in names:
            rate, when = worst[name]
            parts.append("%s %.0f deg/s @%s" % (name, rate, wall(when) if rate >= 1.0 else "-"))
        lines.append("  JRATE   " + " | ".join(parts))
    return "\n".join(lines)


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    paths = []
    for arg in argv[1:]:
        if os.path.isdir(arg):
            paths.extend(sorted(glob.glob(os.path.join(arg, "*.csv"))))
        else:
            paths.extend(glob.glob(arg))
    if not paths:
        print("no CSV found in %s" % " ".join(argv[1:]))
        return 1
    print("rc_ctrl black box report")
    for path in paths:
        print("")
        print(os.path.basename(path))
        try:
            print(analyse(path))
        except Exception as exc:  # keep going on a malformed file
            print("  failed: %s: %s" % (type(exc).__name__, exc))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
