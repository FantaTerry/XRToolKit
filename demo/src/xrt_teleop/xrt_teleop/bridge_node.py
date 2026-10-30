"""Publish XRoboToolkit Quest 3 controller data for the JAKA dual-arm demo."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy

from xrt_teleop.xrt_adapter import (
    DEFAULT_HEADSET_TO_ROBOT,
    joy_payload,
    matrix_3x3,
    normalize_quaternion,
    raw_pose_is_valid,
    smooth_pose,
    transform_pose,
)


class XrtTeleopBridge(Node):
    """Translate XRoboToolkit SDK getters into the legacy ROS teleop contract."""

    def __init__(self):
        super().__init__("xrt_teleop_bridge")

        self.left_target_topic = str(
            self.declare_parameter("left_target_topic", "/rc_ctrl/left_target").value
        )
        self.right_target_topic = str(
            self.declare_parameter("right_target_topic", "/rc_ctrl/right_target").value
        )
        self.button_topic = str(
            self.declare_parameter("button_topic", "/rc_ctrl/button").value
        )
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 125.0).value
        )
        self.smooth_alpha = float(
            self.declare_parameter("smooth_alpha", 0.4).value
        )
        self.pose_scale = float(
            self.declare_parameter("pose_scale", 1.0).value
        )
        self.grip_threshold = float(
            self.declare_parameter("grip_threshold", 0.5).value
        )
        self.input_log_period = float(
            self.declare_parameter("input_log_period", 5.0).value
        )
        self.enable_left = bool(
            self.declare_parameter("enable_left", True).value
        )
        self.enable_right = bool(
            self.declare_parameter("enable_right", True).value
        )
        self.robot_base_frame = str(
            self.declare_parameter("robot_base_frame", "world").value
        )
        self.quest_mac = str(
            self.declare_parameter("quest_mac", "50:99:03:0a:38:78").value
        )

        mapping_values = self.declare_parameter(
            "headset_to_robot", list(DEFAULT_HEADSET_TO_ROBOT)
        ).value
        self.mapping = matrix_3x3(mapping_values)
        self.base_translation = self._vector_parameter(
            "tf_translation", (0.0, 0.0, 0.0), 3
        )
        self.base_rotation = normalize_quaternion(
            self._vector_parameter("tf_rotation", (1.0, 0.0, 0.0, 0.0), 4)
        )

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self.pose_scale <= 0.0:
            raise ValueError("pose_scale must be positive")
        self.smooth_alpha = max(0.0, min(1.0, self.smooth_alpha))
        self.grip_threshold = max(0.0, min(1.0, self.grip_threshold))
        self.input_log_period = max(0.0, self.input_log_period)

        qos = QoSProfile(depth=1)
        self.left_publisher = self.create_publisher(
            PoseStamped, self.left_target_topic, qos
        )
        self.right_publisher = self.create_publisher(
            PoseStamped, self.right_target_topic, qos
        )
        self.button_publisher = self.create_publisher(
            Joy, self.button_topic, qos
        )

        self._filtered = {"left": None, "right": None}
        self._pose_was_valid = {"left": False, "right": False}
        self._last_valid_trigger = {"left": 0.0, "right": 0.0}
        self._last_warning_time = 0.0
        self._last_input_log_time = 0.0
        self._xrt = None
        self._sdk_initialized = False
        self._initialize_sdk()

        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)
        self.get_logger().info(
            "XRoboToolkit bridge started at %.1f Hz: left=%s right=%s buttons=%s"
            % (
                self.publish_rate,
                self.left_target_topic,
                self.right_target_topic,
                self.button_topic,
            )
        )
        self.get_logger().info(
            "Expected Quest 3 MAC %s (diagnostic metadata; PC-service performs "
            "device discovery)" % self.quest_mac
        )

    def _vector_parameter(self, name, default, length):
        values = self.declare_parameter(name, list(default)).value
        try:
            result = tuple(float(value) for value in values)
        except (TypeError, ValueError) as exc:
            raise ValueError("%s must contain %d numeric values" % (name, length)) from exc
        if len(result) != length or not all(math.isfinite(value) for value in result):
            raise ValueError("%s must contain %d finite values" % (name, length))
        return result

    def _initialize_sdk(self):
        try:
            import xrobotoolkit_sdk as xrt
        except ImportError as exc:
            raise RuntimeError(
                "xrobotoolkit_sdk is not installed. Install the XRoboToolkit "
                "PC-Service Python binding before launching this node."
            ) from exc

        try:
            xrt.init()
        except Exception as exc:
            raise RuntimeError(
                "XRoboToolkit SDK initialization failed. Start PC-service and "
                "check that the Quest app is connected."
            ) from exc
        self._xrt = xrt
        self._sdk_initialized = True

    def _safe_value(self, getter_name, default):
        try:
            return getattr(self._xrt, getter_name)()
        except Exception:
            return default

    @staticmethod
    def _finite_float(value, default):
        try:
            result = float(value)
        except (TypeError, ValueError):
            return default
        return result if math.isfinite(result) else default

    def _controller_state(self, side):
        prefix = "left" if side == "left" else "right"
        pose = self._safe_value("get_%s_controller_pose" % prefix, ())
        pose_valid = raw_pose_is_valid(pose)
        grip = self._finite_float(
            self._safe_value("get_%s_grip" % prefix, 0.0), 0.0
        )
        trigger = self._finite_float(
            self._safe_value("get_%s_trigger" % prefix, None),
            self._last_valid_trigger[side],
        )
        axis = self._safe_value("get_%s_axis" % prefix, (0.0, 0.0))
        if pose_valid:
            self._last_valid_trigger[side] = max(0.0, min(1.0, trigger))
        else:
            # Tracking loss must release the arm without commanding the gripper
            # to change state. Keep the last trigger value in the Joy message.
            trigger = self._last_valid_trigger[side]
        return {
            "pose": pose,
            "pose_valid": pose_valid,
            "grip": grip if pose_valid else 0.0,
            "trigger": trigger,
            "axis": axis if pose_valid else (0.0, 0.0),
            "X": (
                pose_valid
                and bool(self._safe_value("get_X_button", False))
                if side == "left" else False
            ),
            "Y": (
                pose_valid
                and bool(self._safe_value("get_Y_button", False))
                if side == "left" else False
            ),
            "A": (
                pose_valid
                and bool(self._safe_value("get_A_button", False))
                if side == "right" else False
            ),
            "B": (
                pose_valid
                and bool(self._safe_value("get_B_button", False))
                if side == "right" else False
            ),
            "menu": pose_valid and bool(
                self._safe_value("get_%s_menu_button" % prefix, False)
            ),
            "axis_click": pose_valid and bool(
                self._safe_value("get_%s_axis_click" % prefix, False)
            ),
        }

    def _pose_message(self, position, quaternion):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.robot_base_frame
        message.pose.position.x = position[0]
        message.pose.position.y = position[1]
        message.pose.position.z = position[2]
        message.pose.orientation.w = quaternion[0]
        message.pose.orientation.x = quaternion[1]
        message.pose.orientation.y = quaternion[2]
        message.pose.orientation.z = quaternion[3]
        return message

    def _publish_pose(self, side, state):
        publisher = self.left_publisher if side == "left" else self.right_publisher
        if not state["pose_valid"]:
            if self._pose_was_valid[side]:
                self._filtered[side] = None
                self.get_logger().warning(
                    "%s controller tracking lost; target publication stopped"
                    % side
                )
            self._pose_was_valid[side] = False
            return

        position, quaternion = transform_pose(
            state["pose"],
            mapping=self.mapping,
            scale=self.pose_scale,
            base_translation=self.base_translation,
            base_rotation=self.base_rotation,
        )
        filtered = smooth_pose(
            self._filtered[side], position, quaternion, self.smooth_alpha
        )
        self._filtered[side] = filtered
        publisher.publish(self._pose_message(*filtered))

        if not self._pose_was_valid[side]:
            self.get_logger().info("%s controller tracking acquired" % side)
        self._pose_was_valid[side] = True

    def _publish(self):
        left = self._controller_state("left") if self.enable_left else {}
        right = self._controller_state("right") if self.enable_right else {}

        if self.enable_left:
            self._publish_pose("left", left)
        if self.enable_right:
            self._publish_pose("right", right)

        axes, buttons = joy_payload(left, right, self.grip_threshold)
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = axes
        joy.buttons = buttons
        # Publish continuously so rc_ctrl receives the current Grip state even
        # when it subscribes several seconds after this bridge while the arms
        # are still powering on and entering servo mode.
        self.button_publisher.publish(joy)

        now = time.monotonic()
        if (
            self.input_log_period > 0.0
            and now - self._last_input_log_time >= self.input_log_period
            and (
                left.get("pose_valid", False)
                or right.get("pose_valid", False)
            )
        ):
            self.get_logger().info(
                "Quest input: left tracked=%s grip=%.3f trigger=%.3f | "
                "right tracked=%s grip=%.3f trigger=%.3f"
                % (
                    left.get("pose_valid", False),
                    left.get("grip", 0.0),
                    left.get("trigger", 0.0),
                    right.get("pose_valid", False),
                    right.get("grip", 0.0),
                    right.get("trigger", 0.0),
                )
            )
            self._last_input_log_time = now

        if (
            not left.get("pose_valid", False)
            and not right.get("pose_valid", False)
            and now - self._last_warning_time > 5.0
        ):
            self.get_logger().warning(
                "No tracked Quest controllers; check PC-service, Quest app, "
                "tracking, and Send"
            )
            self._last_warning_time = now

    def close(self):
        if self._sdk_initialized and self._xrt is not None:
            try:
                self._xrt.close()
            except Exception:
                pass
            self._sdk_initialized = False


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = XrtTeleopBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError, ValueError) as exc:
        if isinstance(exc, (RuntimeError, ValueError)):
            print("[xrt_teleop_bridge] %s" % exc)
            exit_code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
