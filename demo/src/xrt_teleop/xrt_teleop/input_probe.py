"""Print raw XRoboToolkit controller state for teleoperation diagnosis."""

import argparse
import math
import sys
import time

from xrt_teleop.xrt_adapter import raw_pose_is_valid


def _float(value, default=0.0):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _pose_text(pose):
    if not raw_pose_is_valid(pose):
        return "untracked"
    values = [_float(value) for value in pose[:7]]
    return (
        "pos=(%+.3f,%+.3f,%+.3f) quat=(%+.3f,%+.3f,%+.3f,%+.3f)"
        % (
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5],
            values[6],
        )
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Stop run_teleop first, then run this to inspect Quest input."
    )
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args(argv)
    if args.duration < 0.0 or args.interval <= 0.0:
        parser.error("--duration must be >= 0 and --interval must be > 0")

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print(
            "xrobotoolkit_sdk is not installed; run demo/install_xrt_sdk.zsh",
            file=sys.stderr,
        )
        return 1

    try:
        xrt.init()
    except Exception as exc:
        print("XRoboToolkit SDK initialization failed: %s" % exc, file=sys.stderr)
        return 1

    deadline = time.monotonic() + args.duration if args.duration else None
    print(
        "Reading raw XRoboToolkit input. Keep the APK open, Tracking on, "
        "and Send on. Press Ctrl-C to stop."
    )
    try:
        while deadline is None or time.monotonic() < deadline:
            try:
                source_ns = xrt.get_time_stamp_ns()
            except (AttributeError, RuntimeError, TypeError):
                source_ns = "unavailable"
            left_pose = xrt.get_left_controller_pose()
            right_pose = xrt.get_right_controller_pose()
            print(
                "source_ns=%s receipt_mono_s=%.6f | left:  %s grip=%.3f trigger=%.3f | "
                "right: %s grip=%.3f trigger=%.3f"
                % (
                    source_ns,
                    time.monotonic(),
                    _pose_text(left_pose),
                    _float(xrt.get_left_grip()),
                    _float(xrt.get_left_trigger()),
                    _pose_text(right_pose),
                    _float(xrt.get_right_grip()),
                    _float(xrt.get_right_trigger()),
                ),
                flush=True,
            )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            xrt.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
