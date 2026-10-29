"""Print raw XRoboToolkit controller state for teleoperation diagnosis."""

import argparse
import math
import sys
import time

from xrt_teleop.xrt_adapter import raw_pose_is_valid


def _analog_text(sdk, getter_name):
    """Show getter failures separately from a real, successfully read zero."""
    try:
        raw = getattr(sdk, getter_name)()
    except Exception as exc:
        return "invalid(valid=False raw=unavailable error=%s: %s)" % (
            type(exc).__name__, exc)
    try:
        value = float(raw)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            return "invalid(valid=False raw=%r reason=outside_[0,1])" % (raw,)
    except (TypeError, ValueError, OverflowError) as exc:
        return "invalid(valid=False raw=%r error=%s: %s)" % (
            raw, type(exc).__name__, exc)
    return "%.3f(valid=True raw=%r)" % (value, raw)


def _pose_text(pose):
    if not raw_pose_is_valid(pose):
        return "untracked"
    values = [float(value) for value in pose[:7]]
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


def _controller_text(sdk, side):
    try:
        pose = _pose_text(getattr(sdk, "get_%s_controller_pose" % side)())
    except Exception as exc:
        pose = "read_error(%s: %s)" % (type(exc).__name__, exc)
    return "pose=%s grip=%s trigger=%s" % (
        pose,
        _analog_text(sdk, "get_%s_grip" % side),
        _analog_text(sdk, "get_%s_trigger" % side),
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Stop run_teleop first, then run this to inspect Quest input."
    )
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not math.isfinite(args.interval) \
            or args.duration < 0.0 or args.interval <= 0.0:
        parser.error("--duration must be finite and >= 0; --interval must be finite and > 0")

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print(
            "xrobotoolkit_sdk is not installed; run demo/install_xrt_sdk.zsh",
            file=sys.stderr,
        )
        return 1

    print("Python: executable=%s version=%s" % (sys.executable, sys.version.split()[0]))
    print("XRoboToolkit SDK: module=%s file=%s version=%s" % (
        getattr(xrt, "__name__", "xrobotoolkit_sdk"),
        getattr(xrt, "__file__", "unavailable"),
        getattr(xrt, "__version__", "unavailable"),
    ))
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
            except Exception as exc:
                source_ns = "unavailable(%s: %s)" % (type(exc).__name__, exc)
            print(
                "source_ns=%s receipt_mono_s=%.6f | left: %s | right: %s"
                % (
                    source_ns,
                    time.monotonic(),
                    _controller_text(xrt, "left"),
                    _controller_text(xrt, "right"),
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
