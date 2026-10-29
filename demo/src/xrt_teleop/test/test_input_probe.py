"""Exercise the operator's raw-input probe without ROS, SDK init or devices."""

import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

from xrt_teleop import input_probe


class FakeSdk(types.ModuleType):
    def __init__(self):
        super().__init__("xrobotoolkit_sdk")
        self.__file__ = "/offline/fake/xrobotoolkit_sdk.so"
        self.__version__ = "offline-test"
        self.init = Mock()
        self.close = Mock()
        self.get_time_stamp_ns = Mock(return_value=123456)
        pose = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
        self.get_left_controller_pose = Mock(return_value=pose)
        self.get_right_controller_pose = Mock(return_value=pose)
        self.get_left_grip = Mock(return_value=0.0)
        self.get_right_grip = Mock(return_value=0.9)
        self.get_left_trigger = Mock(return_value=0.7)
        self.get_right_trigger = Mock(return_value=0.2)


class InputProbeTest(unittest.TestCase):
    def run_once(self, sdk):
        now = [0.0]

        def sleep(seconds):
            now[0] += seconds

        output = io.StringIO()
        errors = io.StringIO()
        with patch.dict(sys.modules, {"xrobotoolkit_sdk": sdk}), \
                patch.object(input_probe.time, "monotonic", side_effect=lambda: now[0]), \
                patch.object(input_probe.time, "sleep", side_effect=sleep), \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
            result = input_probe.main(["--duration", "0.01", "--interval", "0.02"])
        return result, output.getvalue(), errors.getvalue()

    def test_main_keeps_reference_getters_and_real_zero_valid(self):
        sdk = FakeSdk()
        result, output, errors = self.run_once(sdk)
        self.assertEqual(result, 0)
        self.assertEqual(errors, "")
        self.assertIn("grip=0.000(valid=True raw=0.0)", output)
        self.assertIn("grip=0.900(valid=True raw=0.9)", output)
        self.assertIn("trigger=0.700(valid=True raw=0.7)", output)
        self.assertIn("trigger=0.200(valid=True raw=0.2)", output)
        self.assertIn("source_ns=123456", output)
        for name in ("init", "close", "get_left_controller_pose", "get_right_controller_pose",
                     "get_left_grip", "get_right_grip", "get_left_trigger", "get_right_trigger"):
            getattr(sdk, name).assert_called_once_with()

    def test_invalid_analog_is_visible_and_never_a_zero_sample(self):
        for value in (float("nan"), float("inf"), -0.01, 1.01, "oops", None):
            with self.subTest(value=value):
                sdk = FakeSdk()
                sdk.get_left_grip.return_value = value
                result, output, _errors = self.run_once(sdk)
                self.assertEqual(result, 0)
                self.assertIn("grip=invalid(valid=False raw=%r" % value, output)
                self.assertNotIn("grip=0.000", output)
                self.assertIn("grip=0.900(valid=True", output)

    def test_getter_failures_are_independent_and_sdk_is_closed(self):
        sdk = FakeSdk()
        sdk.get_left_controller_pose.side_effect = RuntimeError("left pose failed")
        sdk.get_left_grip.side_effect = RuntimeError("left grip failed")
        sdk.get_left_trigger.side_effect = ValueError("left trigger failed")
        sdk.get_time_stamp_ns.side_effect = RuntimeError("timestamp failed")
        result, output, _errors = self.run_once(sdk)
        self.assertEqual(result, 0)
        self.assertIn("pose=read_error(RuntimeError: left pose failed)", output)
        self.assertIn("grip=invalid(valid=False raw=unavailable error=RuntimeError: left grip failed)", output)
        self.assertIn("trigger=invalid(valid=False raw=unavailable error=ValueError: left trigger failed)", output)
        self.assertIn("source_ns=unavailable(RuntimeError: timestamp failed)", output)
        self.assertIn("right: pose=pos=", output)
        self.assertIn("grip=0.900(valid=True", output)
        sdk.close.assert_called_once_with()

    def test_startup_identifies_python_and_sdk_import_location(self):
        _result, output, _errors = self.run_once(FakeSdk())
        self.assertIn(sys.executable, output)
        self.assertIn("module=xrobotoolkit_sdk", output)
        self.assertIn("file=/offline/fake/xrobotoolkit_sdk.so", output)
        self.assertIn("version=offline-test", output)

    def test_import_has_no_sdk_or_robot_side_effects(self):
        source = Path(input_probe.__file__)
        spec = importlib.util.spec_from_file_location("offline_input_probe_import", source)
        module = importlib.util.module_from_spec(spec)
        sdk = FakeSdk()
        with patch.dict(sys.modules, {"xrobotoolkit_sdk": sdk}):
            spec.loader.exec_module(module)
        sdk.init.assert_not_called()
        sdk.get_left_grip.assert_not_called()
        sdk.close.assert_not_called()


if __name__ == "__main__":
    unittest.main()
