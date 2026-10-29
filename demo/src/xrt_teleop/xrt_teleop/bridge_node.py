"""Publish XRoboToolkit Quest 3 controller data for the JAKA dual-arm demo."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int64

from xrt_teleop.xrt_adapter import (
    DEFAULT_HEADSET_TO_ROBOT,
    SourceFreshness,
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
        self.source_timeout_s = float(
            self.declare_parameter("source_timeout_s", 0.2).value
        )
        self.require_source_timestamp = bool(
            self.declare_parameter("require_source_timestamp", True).value
        )
        self.left_tracking_topic = str(self.declare_parameter(
            "left_tracking_topic", "/rc_ctrl/left_tracking"
        ).value)
        self.right_tracking_topic = str(self.declare_parameter(
            "right_tracking_topic", "/rc_ctrl/right_tracking"
        ).value)
        self.left_input_state_topic = str(self.declare_parameter(
            "left_input_state_topic", "/rc_ctrl/left_input_state"
        ).value)
        self.right_input_state_topic = str(self.declare_parameter(
            "right_input_state_topic", "/rc_ctrl/right_input_state"
        ).value)
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
        self.tracking_publishers = {
            "left": self.create_publisher(Bool, self.left_tracking_topic, qos),
            "right": self.create_publisher(Bool, self.right_tracking_topic, qos),
        }
        self.input_state_publishers = {
            "left": self.create_publisher(Int64, self.left_input_state_topic, qos),
            "right": self.create_publisher(Int64, self.right_input_state_topic, qos),
        }

        self._filtered = {"left": None, "right": None}
        self._pose_was_valid = {"left": False, "right": False}
        self._input_state_value = {"left": 0, "right": 0}
        self._input_was_valid = {"left": False, "right": False}
        self._last_valid_trigger = {"left": 0.0, "right": 0.0}
        self._last_valid_grip = {"left": 0.0, "right": 0.0}
        self._last_warning_time = 0.0
        self._last_input_log_time = 0.0
        self._xrt = None
        self._sdk_initialized = False
        self._source = SourceFreshness(
            self.source_timeout_s, self.require_source_timestamp
        )
        self._source_stamp = None
        self._cached_states = {"left": {}, "right": {}}
        self._poll_count = 0
        self._source_count = 0
        self._pose_counts = {"left": 0, "right": 0}
        self._rate_started_s = time.monotonic()
        self._initialize_sdk()
        self._has_source_timestamp = callable(
            getattr(self._xrt, "get_time_stamp_ns", None)
        )
        if not self._has_source_timestamp:
            self.get_logger().warning(
                "SDK lacks get_time_stamp_ns; require_source_timestamp=%s. "
                "Receipt-only fallback cannot detect cached nonzero poses."
                % self.require_source_timestamp
            )
        self.get_logger().info(
            "Source freshness: timestamp=%s timeout=%.3fs; controller tracking "
            "uses valid-pose sentinel (no per-controller tracking flag in the "
            "reviewed Python API). SDK getters do not provide an atomic snapshot."
            % (self._has_source_timestamp, self.source_timeout_s)
        )

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

    def _read_analog(self, getter_name, previous):
        """Return a numeric sample and independent getter validity.

        Invalid values must not become a successful zero-valued release or a
        saturated Trigger command. Retaining the previous numeric value is
        diagnostic/legacy compatibility only; validity bits prohibit its use.
        """
        try:
            result = float(getattr(self._xrt, getter_name)())
            valid = math.isfinite(result) and 0.0 <= result <= 1.0
        except (AttributeError, TypeError, ValueError, OverflowError):
            valid = False
        except Exception:
            valid = False
        return (result, True) if valid else (previous, False)

    def _controller_state(self, side):
        prefix = "left" if side == "left" else "right"
        pose = self._safe_value("get_%s_controller_pose" % prefix, ())
        pose_valid = raw_pose_is_valid(pose)
        grip, grip_valid = self._read_analog(
            "get_%s_grip" % prefix, self._last_valid_grip[side]
        )
        trigger, trigger_valid = self._read_analog(
            "get_%s_trigger" % prefix, self._last_valid_trigger[side]
        )
        axis = self._safe_value("get_%s_axis" % prefix, (0.0, 0.0))
        if pose_valid and grip_valid:
            self._last_valid_grip[side] = grip
        if pose_valid and trigger_valid:
            self._last_valid_trigger[side] = trigger
        if not pose_valid:
            # Tracking loss must release the arm without commanding the gripper
            # to change state. Keep the last trigger value in the Joy message.
            trigger = self._last_valid_trigger[side]
        return {
            "pose": pose,
            "pose_valid": pose_valid,
            "grip": grip if pose_valid else 0.0,
            "grip_valid": grip_valid,
            "trigger": trigger,
            "trigger_valid": trigger_valid,
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
        # This is the local receipt time of a new source timestamp. Source
        # clock epoch is unspecified and must not be cast to a ROS timestamp.
        message.header.stamp = self._source_stamp
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
        if not state["new_frame"]:
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
        self._pose_counts[side] += 1

        if not self._pose_was_valid[side]:
            self.get_logger().info("%s controller tracking acquired" % side)
        self._pose_was_valid[side] = True

    def _publish(self):
        now = time.monotonic()
        self._poll_count += 1
        source_ns = self._safe_value("get_time_stamp_ns", None)
        fresh, new_frame = self._source.observe(
            source_ns, now, self._has_source_timestamp
        )
        if fresh and new_frame:
            # Read each getter once per source frame. In particular, do not
            # repeatedly smooth/re-stamp the same cached pose at timer rate.
            self._source_stamp = self.get_clock().now().to_msg()
            self._source_count += 1
            for side, enabled in (("left", self.enable_left), ("right", self.enable_right)):
                self._cached_states[side] = self._controller_state(side) if enabled else {}
        states = {}
        for side, enabled in (("left", self.enable_left), ("right", self.enable_right)):
            state = dict(self._cached_states[side]) if enabled else {}
            state["pose_valid"] = fresh and bool(state.get("pose_valid", False))
            state["new_frame"] = fresh and new_frame
            if not state["pose_valid"]:
                # Neutral Grip on disconnect; retain the last accepted trigger
                # so loss of tracking does not open a closed gripper.
                state.update(grip=0.0, trigger=self._last_valid_trigger[side],
                             grip_valid=False, trigger_valid=False,
                             axis=(0.0, 0.0), X=False, Y=False, A=False, B=False,
                             menu=False, axis_click=False)
            states[side] = state
            tracking = Bool()
            tracking.data = state["pose_valid"]
            self.tracking_publishers[side].publish(tracking)
            # This source-issued barrier is authoritative for recovery. Bool
            # alone cannot fence complete pre-loss frames still in transit.
            # Positive: first recovered frame's ROS stamp, fixed throughout
            # this healthy generation. Negative: first observed loss ROS stamp.
            # The value is repeated so a lost transition cannot remove fencing.
            valid = state["pose_valid"]
            if self._input_state_value[side] == 0 or valid != self._input_was_valid[side]:
                stamp = self._source_stamp if valid else self.get_clock().now().to_msg()
                stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
                self._input_state_value[side] = stamp_ns if valid else -stamp_ns
                self._input_was_valid[side] = valid
            input_state = Int64()
            input_state.data = self._input_state_value[side]
            self.input_state_publishers[side].publish(input_state)
        left, right = states["left"], states["right"]

        if self.enable_left:
            self._publish_pose("left", left)
        if self.enable_right:
            self._publish_pose("right", right)

        axes, buttons = joy_payload(left, right, self.grip_threshold)
        joy = Joy()
        joy.header.stamp = (
            self._source_stamp if fresh and self._source_stamp is not None
            else self.get_clock().now().to_msg()
        )
        joy.axes = axes
        joy.buttons = buttons
        # Publish continuously so rc_ctrl receives the current Grip state even
        # when it subscribes several seconds after this bridge while the arms
        # are still powering on and entering servo mode.
        self.button_publisher.publish(joy)

        if (
            self.input_log_period > 0.0
            and now - self._last_input_log_time >= self.input_log_period
        ):
            elapsed = max(now - self._rate_started_s, 1e-9)
            age = (
                now - self._source.last_advance_s
                if self._source.last_advance_s is not None else float("inf")
            )
            self.get_logger().info(
                "Quest input: left tracked=%s grip=%.3f(valid=%s) trigger=%.3f(valid=%s) | "
                "right tracked=%s grip=%.3f(valid=%s) trigger=%.3f(valid=%s) | source_ns=%s "
                "receipt_mono_s=%s age_s=%.3f status=%s "
                "poll_hz=%.1f source_hz=%.1f pose_hz=(%.1f,%.1f)"
                % (
                    left.get("pose_valid", False),
                    left.get("grip", 0.0),
                    left.get("grip_valid", False),
                    left.get("trigger", 0.0),
                    left.get("trigger_valid", False),
                    right.get("pose_valid", False),
                    right.get("grip", 0.0),
                    right.get("grip_valid", False),
                    right.get("trigger", 0.0),
                    right.get("trigger_valid", False),
                    self._source.last_source_ns,
                    self._source.last_advance_s,
                    age,
                    self._source.reason,
                    self._poll_count / elapsed,
                    self._source_count / elapsed,
                    self._pose_counts["left"] / elapsed,
                    self._pose_counts["right"] / elapsed,
                )
            )
            self._last_input_log_time = now
            self._rate_started_s = now
            self._poll_count = self._source_count = 0
            self._pose_counts = {"left": 0, "right": 0}

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
