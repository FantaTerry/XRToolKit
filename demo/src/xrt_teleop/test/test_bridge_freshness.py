"""Exercise the real bridge methods with ROS message/publisher and SDK fakes.

No ROS installation, device discovery, SDK initialization or sockets are used.
"""

import copy
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


class PoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(w=0.0, x=0.0, y=0.0, z=0.0),
        )


class Joy:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.axes = []
        self.buttons = []


class Bool:
    def __init__(self):
        self.data = False


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class FakeNode:
    overrides = {}

    def __init__(self, _name):
        self.clock_ns = 0
        self.logs = []

    def declare_parameter(self, name, default):
        return types.SimpleNamespace(value=self.overrides.get(name, default))

    def create_publisher(self, *_args):
        return Publisher()

    def create_timer(self, *_args):
        return object()

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            to_msg=lambda: types.SimpleNamespace(
                sec=self.clock_ns // 1000000000,
                nanosec=self.clock_ns % 1000000000,
            )
        ))

    def get_logger(self):
        return types.SimpleNamespace(
            info=self.logs.append, warning=self.logs.append
        )


def load_bridge():
    modules = {}
    for name in ("rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs",
                 "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg",
                 "std_msgs", "std_msgs.msg"):
        modules[name] = types.ModuleType(name)
    modules["rclpy.node"].Node = FakeNode
    modules["rclpy.qos"].QoSProfile = lambda **kwargs: kwargs
    modules["geometry_msgs.msg"].PoseStamped = PoseStamped
    modules["sensor_msgs.msg"].Joy = Joy
    modules["std_msgs.msg"].Bool = Bool
    source = Path(__file__).parents[1] / "xrt_teleop" / "bridge_node.py"
    spec = importlib.util.spec_from_file_location("offline_xrt_bridge", source)
    bridge = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(bridge)
    return bridge


bridge_module = load_bridge()


class FakeSdk:
    def __init__(self):
        self.timestamp = 1000
        self.left_pose = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
        self.right_pose = list(self.left_pose)

    def get_time_stamp_ns(self):
        return self.timestamp

    def __getattr__(self, name):
        if name == "get_left_controller_pose":
            return lambda: self.left_pose
        if name == "get_right_controller_pose":
            return lambda: self.right_pose
        if name.endswith("_grip"):
            return lambda: 0.9
        if name.endswith("_trigger"):
            return lambda: 0.7
        if name.endswith("_axis"):
            return lambda: (0.1, -0.1)
        return lambda: False


class BridgeFreshnessTest(unittest.TestCase):
    def make_bridge(self, sdk=None, **overrides):
        sdk = sdk or FakeSdk()
        FakeNode.overrides = overrides

        def initialize(node):
            node._xrt = sdk
            node._sdk_initialized = True

        with patch.object(bridge_module.XrtTeleopBridge, "_initialize_sdk", initialize), \
                patch.object(bridge_module.time, "monotonic", return_value=0.0):
            node = bridge_module.XrtTeleopBridge()
        return node, sdk

    def tick(self, node, now):
        node.clock_ns = 1000000000 + round(now * 1000000000)
        with patch.object(bridge_module.time, "monotonic", return_value=now):
            node._publish()

    def test_cached_pose_not_restamped_or_resmoothed(self):
        node, sdk = self.make_bridge()
        self.tick(node, 0.0)
        initial_filter = copy.deepcopy(node._filtered)
        initial_stamp = node.button_publisher.messages[-1].header.stamp
        # The getters changing without timestamp progress must not be consumed.
        sdk.left_pose[0] += 1.0
        self.tick(node, 0.1)
        self.assertEqual(len(node.left_publisher.messages), 1)
        self.assertEqual(node._filtered, initial_filter)
        self.assertEqual(node.button_publisher.messages[-1].header.stamp, initial_stamp)
        self.assertTrue(node.tracking_publishers["left"].messages[-1].data)

    def test_stationary_pose_with_new_source_frames_keeps_tracking(self):
        node, sdk = self.make_bridge()
        for frame in range(100):
            sdk.timestamp += 1
            self.tick(node, frame * 0.01)
        self.assertEqual(len(node.left_publisher.messages), 100)
        self.assertTrue(node.tracking_publishers["left"].messages[-1].data)
        self.assertEqual(node.button_publisher.messages[-1].axes[0:4], [0.9, 0.9, 0.7, 0.7])

    def test_source_timeout_releases_grip_and_preserves_gripper(self):
        node, _sdk = self.make_bridge()
        self.tick(node, 0.0)
        self.tick(node, 0.201)
        self.assertEqual(len(node.left_publisher.messages), 1)
        self.assertFalse(node.tracking_publishers["left"].messages[-1].data)
        self.assertFalse(node.tracking_publishers["right"].messages[-1].data)
        self.assertEqual(node.button_publisher.messages[-1].axes[0:4], [0.0, 0.0, 0.7, 0.7])
        self.assertIsNone(node._filtered["left"])

    def test_reacquisition_discards_old_filter_reference(self):
        node, sdk = self.make_bridge()
        self.tick(node, 0.0)
        self.tick(node, 0.201)
        sdk.timestamp += 1
        sdk.left_pose[0] = 2.0
        self.tick(node, 0.21)
        self.assertAlmostEqual(node.left_publisher.messages[-1].pose.position.y, -2.0)
        self.assertTrue(node.tracking_publishers["left"].messages[-1].data)

    def test_invalid_one_side_releases_only_that_arm(self):
        node, sdk = self.make_bridge()
        self.tick(node, 0.0)
        sdk.timestamp += 1
        sdk.left_pose = [0.0] * 7
        self.tick(node, 0.01)
        self.assertFalse(node.tracking_publishers["left"].messages[-1].data)
        self.assertTrue(node.tracking_publishers["right"].messages[-1].data)
        self.assertEqual(node.button_publisher.messages[-1].axes[0:4], [0.0, 0.9, 0.7, 0.7])

    def test_missing_timestamp_default_prevents_target_publication(self):
        sdk = FakeSdk()
        sdk.get_time_stamp_ns = None
        node, _sdk = self.make_bridge(sdk)
        self.tick(node, 0.0)
        self.assertEqual(len(node.left_publisher.messages), 0)
        self.assertEqual(node.button_publisher.messages[-1].axes[0:2], [0.0, 0.0])
        self.assertFalse(node.tracking_publishers["left"].messages[-1].data)

    def test_explicit_receipt_fallback_and_runtime_timestamp_failure(self):
        sdk = FakeSdk()
        sdk.get_time_stamp_ns = None
        node, _sdk = self.make_bridge(sdk, require_source_timestamp=False)
        self.tick(node, 0.0)
        self.assertEqual(len(node.left_publisher.messages), 1)
        # A callable getter that later returns an invalid timestamp fails closed.
        node, sdk = self.make_bridge(require_source_timestamp=False)
        self.tick(node, 0.0)
        sdk.timestamp = None
        self.tick(node, 0.01)
        self.assertFalse(node.tracking_publishers["left"].messages[-1].data)
        self.assertEqual(len(node.left_publisher.messages), 1)


if __name__ == "__main__":
    unittest.main()
