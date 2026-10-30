#include <algorithm>
#include <iostream>
#include <string>
#include <map>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>
#include <memory>
#include <mutex>
#include <cstring>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/orientation_math.hpp"
#include "jaka_driver/pgi140_gripper.hpp"
#include "jaka_driver/servo_guard.hpp"

using namespace std;

// SDK return status -> readable message
map<int, string> mapErr = {
    {2,   "ERR_FUCTION_CALL_ERROR"},
    {-1,  "ERR_INVALID_HANDLER"},
    {-2,  "ERR_INVALID_PARAMETER"},
    {-3,  "ERR_COMMUNICATION_ERR"},
    {-4,  "ERR_KINE_INVERSE_ERR"},
    {-5,  "ERR_EMERGENCY_PRESSED"},
    {-6,  "ERR_NOT_POWERED"},
    {-7,  "ERR_NOT_ENABLED"},
    {-8,  "ERR_DISABLE_SERVOMODE"},
    {-9,  "ERR_NOT_OFF_ENABLE"},
    {-10, "ERR_PROGRAM_IS_RUNNING"},
    {-11, "ERR_CANNOT_OPEN_FILE"},
    {-12, "ERR_MOTION_ABNORMAL"}
};

// Single-arm teleop node. The JAKA SDK holds ONE global connection per process
// (libjakaAPI.so's `robot` shared_ptr), so a process can only drive a single
// robot. This node controls exactly one arm, chosen by the `arm_name` parameter;
// the launch file starts two instances (one per arm).
JAKAZuRobot robot;

// Controller pose: translation (mm) + orientation (quaternion). Orientation is
// kept as a quaternion end-to-end so the per-cycle incremental rotation is
// computed correctly; RPY is derived only for the final servo_p delta.
struct CtrlPose {
    CartesianTran tran;   // mm
    Quaternion   quat;    // s == w (SDK scalar == ROS w)
};

// Teleop state: whether the grip is currently held, the controller pose captured
// at the press instant, and the latest controller pose.
//
// `gripped` is the cross-thread gate between the background re-arm thread and the
// target callback: re-arm writes ctrl_origin/ctrl_prev BEFORE setting gripped=true,
// and target_callback only reads them AFTER observing gripped==true, so the atomic
// store/load establishes the needed happens-before (no data race). While re-arming
// (gripped false) the target callback does nothing, so the SDK is never touched
// concurrently by servo_p and the re-arm sequence.
std::atomic<bool> gripped{false};
std::atomic<bool> rearming{false};
std::atomic<bool> hold_stream_enabled{false};
int hold_stream_failure_count = 0;
// Latest grip state, mirrored here so the re-arm thread can check whether the
// grip is STILL held when it finishes (a press-then-release during re-arm must
// not leave the arm tracking).
std::atomic<bool> grip_now{false};
std::thread rearm_thread;
bool debug = false;
CtrlPose ctrl_origin;   // controller pose at grip press (origin)
CtrlPose ctrl_prev;     // last controller pose we commanded
Quaternion commanded_rotation_delta = jaka_teleop::quat_identity();
// Latest target pose (stored by value under a mutex: it is written by the target
// callback every cycle and read by the re-arm thread at grip press).
geometry_msgs::msg::PoseStamped latest_pose;
std::mutex latest_mtx;
// Whether the arm connected successfully at startup (login_in == 0). A failed
// arm is skipped for power_on/enable/servo.
bool arm_ok = false;
std::string arm_name = "left";
int grip_axis = 0;           // index into Joy.axes: 0 = leftGrip, 1 = rightGrip
int gripper_axis = 2;        // index trigger: 2 = leftTrig, 3 = rightTrig
bool gripper_use_button = false;
int gripper_button_index = 0;
bool gripper_linear = true; // true = linear (analog axes), false = digital (button)
double gripper_trigger_deadband = 0.02; // minimal change to send new Grip()
double prev_gripper_trigger = 0.0;
std::unique_ptr<pgi140::Pgi140Gripper> gripper;
bool use_rpy_ctrl = true;   // true = 6D teleop, false = position-only fallback
double rotation_scale = 0.5; // relative hand rotation mapped to the robot
double rotation_gain = 0.5; // fraction of remaining controller rotation per cycle
double rotation_deadband = 0.005;  // rad
double rotation_max_step = 0.0015; // rad per 8 ms servo cycle (~10.7 deg/s)
int rotation_sign = 1;             // 1 or -1 for all robot rotation axes
double grip_press_threshold = 0.6;
double grip_release_threshold = 0.4;
bool enable_y_limit = true; // true = clamp arm TCP y to its workspace range
bool arm_is_right = false;  // mirror of arm_name == "right"
double arm_tcp_y_at_grip = 0.0; // arm's absolute TCP y captured at grip press (mm)
bool arm_tcp_valid = false;     // whether the grip-time TCP read succeeded

// ---------------------------------------------------------------------------
// Conservative servo governors. They sit between the hand mapping and
// servo_p(INCR) and can only ever make the command smaller or later:
//   leash : clamp the accumulated hand offset to a sphere round the grip point
//   accel : bound how fast the per-cycle increment may change (mm/s^2)
//   gate  : withhold new increments while the arm is measurably behind
// Each one is switchable at runtime with `ros2 param set`; 0 disables it.
// The gate is closed loop, so it stays off unless the measured SDK feedback
// calls fit comfortably inside one 8 ms servo cycle.
// ---------------------------------------------------------------------------
double servo_leash_radius_mm = 600.0;
double servo_trans_accel_mm_s2 = 1500.0;
double servo_max_track_error_mm = 10.0;
double servo_track_deadman_s = 2.0;
double servo_feedback_budget_us = 200.0;   // average us per 8 ms cycle
bool servo_feedback_enabled = true;
// Optional wrist-singularity slow down. The wrist is singular when J5 is 0 or
// +-180 deg, and there the same Cartesian step asks for a much larger joint
// step. Disabled by default: it is a mitigation, not a diagnosis.
bool servo_singularity_slowdown = false;
double servo_singularity_threshold_deg = 15.0;
double servo_singularity_min_scale = 0.35;
bool actual_joints_valid = false;
double actual_joints_time = 0.0;
// The controller opens one step_num*8 ms interpolation window per servo_p call.
// A second command inside that window is reported as a continuous servo command
// and the step is dropped, so the message driven path keeps a minimum spacing.
double servo_min_period_s = 0.0085;
bool servo_timer_driven = false;  // true: one command per window from the timer
double last_servo_send = 0.0;
constexpr double PI_D = 3.14159265358979323846;
// Max translation per 8 ms servo cycle. The old 5.0 mm/cycle value is 625 mm/s,
// not the 125 mm/s its comment claimed, and that slew is what pushed joint 3
// past the controller's position-deviation window near the offending pose.
// 1.0 mm/cycle = 125 mm/s; live-tunable with servo_max_tran_step_mm.
double servo_max_tran_step = 1.0;
jaka_guard::AccelState guard_accel;

constexpr double SERVO_CYCLE_S = 0.008;
CartesianPose arm_tcp_at_grip{};   // commanded TCP captured at the Grip press
CartesianPose actual_tcp{};        // get_actual_tcp_position()
JointValue actual_joints{};        // get_actual_joint_position()
bool actual_tcp_valid = false;
double actual_tcp_time = 0.0;
double last_feedback_us = 0.0;
bool feedback_polling = false;
int feedback_stride = 1;
unsigned long feedback_tick = 0;
bool status_polling = false;
int status_stride = 13;
unsigned long status_tick = 0;
int last_errcode = 0;
double track_hold_since = -1.0;
bool track_gate_gave_up = false;
rclcpp::Node::SharedPtr g_node;

std::ofstream blackbox;
double blackbox_flush_time = 0.0;

bool servo_hold(JAKAZuRobot &robot);
bool servo_send_allowed(double now);

double steady_seconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Black box: one row per servo cycle plus event rows, so an alarm can be
// replayed afterwards. It never changes control behaviour and every failure to
// open or write it is non-fatal.
void blackbox_header()
{
    blackbox << "t,kind,reason,incr_x,incr_y,incr_z,off_x,off_y,off_z,"
                "cmd_x,cmd_y,cmd_z,act_x,act_y,act_z,fb_valid,j1,j2,j3,j4,j5,j6,"
                "errcode,ret,held,fb_us\n";
}

void blackbox_line(const char *kind, const char *reason, const CartesianPose &incr,
                   double ox, double oy, double oz,
                   double cx, double cy, double cz, int ret, bool held)
{
    if (!blackbox.is_open())
        return;
    const double t = steady_seconds();
    blackbox << t << ',' << kind << ',' << reason
             << ',' << incr.tran.x << ',' << incr.tran.y << ',' << incr.tran.z
             << ',' << ox << ',' << oy << ',' << oz
             << ',' << cx << ',' << cy << ',' << cz
             << ',' << actual_tcp.tran.x << ',' << actual_tcp.tran.y << ',' << actual_tcp.tran.z
             << ',' << (actual_tcp_valid ? 1 : 0);
    for (int i = 0; i < 6; ++i)
        blackbox << ',' << actual_joints.jVal[i];
    blackbox << ',' << last_errcode << ',' << ret << ',' << (held ? 1 : 0)
             << ',' << last_feedback_us << '\n';
    if (t - blackbox_flush_time > 1.0)
    {
        blackbox.flush();
        blackbox_flush_time = t;
    }
}

template <class F>
double measure_us(F call, int repeats)
{
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < repeats; ++i)
        call();
    const auto dt = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - t0).count();
    return dt / static_cast<double>(repeats);
}

// Measure the real cost of every candidate feedback getter once, then pick the
// cheapest one that actually returns data. get_actual_* is not available on
// every controller, and get_robot_status() returns TCP, the six joints and the
// error code in a single call, so it is often both cheaper and more complete.
// 0 = none, 1 = get_actual_tcp_position + get_actual_joint_position,
// 2 = get_robot_status.
int feedback_source = 0;

void calibrate_servo_feedback()
{
    feedback_polling = false;
    status_polling = false;
    feedback_source = 0;
    if (!servo_feedback_enabled)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] servo_feedback_enabled=false: tracking gate and alarm log are off",
                    arm_name.c_str());
        return;
    }

    CartesianPose tcp{};
    JointValue joints{};
    RobotStatus big{};
    RobotStatus_simple status{};
    const bool tcp_ok = (robot.get_actual_tcp_position(&tcp) == 0);
    const bool joint_ok = (robot.get_actual_joint_position(&joints) == 0);
    const bool big_ok = (robot.get_robot_status(&big) == 0);
    const double tcp_us = measure_us([&]() {robot.get_actual_tcp_position(&tcp);}, 20);
    const double joint_us = measure_us([&]() {robot.get_actual_joint_position(&joints);}, 20);
    const double big_us = measure_us([&]() {robot.get_robot_status(&big);}, 10);
    const double status_us = measure_us([&]() {robot.get_robot_status_simple(&status);}, 10);
    const double pair_us = tcp_us + joint_us;

    double motion_us = 0.0;
    if (big_ok && (!tcp_ok || !joint_ok || big_us <= pair_us))
    {
        feedback_source = 2;
        motion_us = big_us;
    }
    else if (tcp_ok && joint_ok)
    {
        feedback_source = 1;
        motion_us = pair_us;
    }

    // A blocking getter inside the servo thread delays the next servo_p call, so
    // the decimation is chosen for the average overhead per cycle, not for a
    // "looks fast enough" threshold.
    feedback_stride = std::max(1, std::min(8,
        static_cast<int>(std::ceil(motion_us / servo_feedback_budget_us))));
    feedback_polling = (feedback_source != 0) && (motion_us <= 4000.0);
    status_stride = std::max(13, std::min(200,
        static_cast<int>(std::ceil(status_us / 100.0))));
    status_polling = (status_us <= 10000.0);

    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] feedback calibration: actual_tcp=%.0fus(ok=%d) actual_joints=%.0fus(ok=%d) "
                "robot_status=%.0fus(ok=%d) simple=%.0fus",
                arm_name.c_str(), tcp_us, tcp_ok ? 1 : 0, joint_us, joint_ok ? 1 : 0,
                big_us, big_ok ? 1 : 0, status_us);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] feedback source=%s every %d cycle(s) (%.0f us/cycle), "
                "gate=%s, alarm log=%s every %d cycle(s)",
                arm_name.c_str(),
                feedback_source == 2 ? "robot_status" : (feedback_source == 1 ? "actual_*" : "none"),
                feedback_stride, feedback_polling ? motion_us / feedback_stride : 0.0,
                feedback_polling ? "on" : "off",
                status_polling ? "on" : "off", status_stride);
    if (!feedback_polling)
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] no affordable motion feedback (robot_status=%.0fus ok=%d, "
                    "actual=%.0fus ok=%d/%d); tracking gate and singularity slow down "
                    "stay off", arm_name.c_str(), big_us, big_ok ? 1 : 0,
                    pair_us, tcp_ok ? 1 : 0, joint_ok ? 1 : 0);
}

// Poll the measured state and the controller alarm code, at the decimation the
// calibration chose. Called from the 8 ms timer, in the same thread as servo_p.
void poll_servo_feedback()
{
    if (feedback_polling && (++feedback_tick % static_cast<unsigned long>(feedback_stride) == 0))
    {
        const double t0 = steady_seconds();
        if (feedback_source == 2)
        {
            RobotStatus st{};
            if (robot.get_robot_status(&st) == 0)
            {
                actual_tcp.tran.x = st.cartesiantran_position[0];
                actual_tcp.tran.y = st.cartesiantran_position[1];
                actual_tcp.tran.z = st.cartesiantran_position[2];
                actual_tcp.rpy.rx = st.cartesiantran_position[3];
                actual_tcp.rpy.ry = st.cartesiantran_position[4];
                actual_tcp.rpy.rz = st.cartesiantran_position[5];
                for (int i = 0; i < 6; ++i)
                    actual_joints.jVal[i] = st.joint_position[i];
                actual_tcp_valid = true;
                actual_joints_valid = true;
                actual_tcp_time = actual_joints_time = steady_seconds();
            }
            else
            {
                actual_tcp_valid = false;
                actual_joints_valid = false;
            }
        }
        else if (feedback_source == 1)
        {
            CartesianPose tcp{};
            JointValue joints{};
            const int ret_tcp = robot.get_actual_tcp_position(&tcp);
            const int ret_joint = robot.get_actual_joint_position(&joints);
            actual_tcp_valid = (ret_tcp == 0);
            if (ret_tcp == 0)
                actual_tcp = tcp;
            actual_joints_valid = (ret_joint == 0);
            if (ret_joint == 0)
                actual_joints = joints;
            actual_tcp_time = actual_joints_time = steady_seconds();
        }
        last_feedback_us = (steady_seconds() - t0) * 1e6;
    }

    if (status_polling && (++status_tick % static_cast<unsigned long>(status_stride) == 0))
    {
        RobotStatus_simple status{};
        if (robot.get_robot_status_simple(&status) == 0)
        {
            const int errcode = status.errcode;
            if (errcode != last_errcode)
            {
                size_t msg_len = 0;
                while (msg_len < sizeof(status.errmsg) && status.errmsg[msg_len] != '\0')
                    ++msg_len;
                const std::string msg(status.errmsg, msg_len);
                CartesianPose none{};
                if (errcode != 0)
                    RCLCPP_ERROR(rclcpp::get_logger("rc_ctrl"),
                                 "[%s] CONTROLLER ALARM errcode=%d (0x%X) msg='%s'",
                                 arm_name.c_str(), errcode, static_cast<unsigned>(errcode),
                                 msg.c_str());
                else
                    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                                "[%s] controller alarm cleared (was %d)",
                                arm_name.c_str(), last_errcode);
                last_errcode = errcode;
                blackbox_line("event", errcode != 0 ? "alarm" : "alarm_cleared", none,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, false);
                if (blackbox.is_open())
                    blackbox.flush();
            }
        }
    }
}

// Re-read the governor limits so they can be changed with `ros2 param set`
// while the node runs. Invalid values are ignored, never applied.
void refresh_governor_params()
{
    if (!g_node)
        return;
    double value = 0.0;
    if (g_node->get_parameter("servo_max_tran_step_mm", value) &&
        std::isfinite(value) && value > 0.0 && value <= 5.0)
        servo_max_tran_step = value;
    if (g_node->get_parameter("servo_leash_radius_mm", value) &&
        std::isfinite(value) && value >= 0.0)
        servo_leash_radius_mm = value;
    if (g_node->get_parameter("servo_trans_accel_mm_s2", value) &&
        std::isfinite(value) && value >= 0.0)
        servo_trans_accel_mm_s2 = value;
    if (g_node->get_parameter("servo_max_track_error_mm", value) &&
        std::isfinite(value) && value >= 0.0)
        servo_max_track_error_mm = value;
    if (g_node->get_parameter("servo_singularity_threshold_deg", value) &&
        std::isfinite(value) && value >= 0.0 && value < 90.0)
        servo_singularity_threshold_deg = value;
    if (g_node->get_parameter("servo_singularity_min_scale", value) &&
        std::isfinite(value) && value > 0.0 && value <= 1.0)
        servo_singularity_min_scale = value;
    bool flag = false;
    if (g_node->get_parameter("servo_singularity_slowdown", flag))
        servo_singularity_slowdown = flag;
    if (g_node->get_parameter("servo_timer_driven", flag))
        servo_timer_driven = flag;
    if (g_node->get_parameter("servo_min_period_ms", value) &&
        std::isfinite(value) && value >= 8.0 && value <= 60.0)
        servo_min_period_s = value / 1000.0;
}

// Angle to the nearest wrist singularity, in radians. JAKA reports joint values
// in radians (jktypes.h), so |J5| itself is the measure once wrapped.
double wrist_singularity_angle()
{
    double j5 = actual_joints.jVal[4];
    if (!std::isfinite(j5))
        return 0.0;
    double wrapped = std::fmod(j5, 2.0 * PI_D);
    if (wrapped < 0.0)
        wrapped += 2.0 * PI_D;
    const double to_zero = std::min(wrapped, 2.0 * PI_D - wrapped);
    const double to_pi = std::fabs(wrapped - PI_D);
    return std::min(to_zero, to_pi);
}

double singularity_scale()
{
    const double threshold = servo_singularity_threshold_deg * PI_D / 180.0;
    if (!(threshold > 0.0))
        return 1.0;
    const double ratio = std::sin(wrist_singularity_angle()) / std::sin(threshold);
    return std::max(servo_singularity_min_scale, std::min(1.0, ratio));
}

// One-shot readout of the controller configuration. This is diagnostic only:
// a wrong payload or a TCP that does not match the real tool is one of the few
// things that produce a load dependent joint deviation alarm.
void log_robot_configuration()
{
    char version[256] = {0};
    int tool_id = -1;
    int user_id = -1;
    PayLoad payload{};
    JointValue joints{};
    const int ret_version = robot.get_sdk_version(version);
    const int ret_tool = robot.get_tool_id(&tool_id);
    const int ret_user = robot.get_user_frame_id(&user_id);
    const int ret_payload = robot.get_payload(&payload);
    const int ret_joints = robot.get_actual_joint_position(&joints);

    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] controller: sdk='%s'(ret=%d) tool=%d(ret=%d) user=%d(ret=%d)",
                arm_name.c_str(), version, ret_version, tool_id, ret_tool,
                user_id, ret_user);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] payload: mass=%.3f kg centroid=(%.1f,%.1f,%.1f) mm (ret=%d); "
                "a zero or stale payload makes gravity compensation wrong and "
                "shows up as a load dependent joint deviation alarm",
                arm_name.c_str(), payload.mass, payload.centroid.x, payload.centroid.y,
                payload.centroid.z, ret_payload);
    if (ret_joints == 0)
    {
        actual_joints = joints;
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] joints(rad): %.4f %.4f %.4f %.4f %.4f %.4f -> J5 is %.1f deg "
                    "from the wrist singularity",
                    arm_name.c_str(), joints.jVal[0], joints.jVal[1], joints.jVal[2],
                    joints.jVal[3], joints.jVal[4], joints.jVal[5],
                    wrist_singularity_angle() * 180.0 / PI_D);
    }
}

// Convert a PoseStamped (position in meters + quaternion) into a controller pose
// (translation in mm + quaternion). No RPY conversion here — the RPY delta is
// computed from quaternions in servo_cartesian().
bool pose_to_ctrl(const geometry_msgs::msg::PoseStamped &msg,
                  CtrlPose &pose)
{
    // ROS uses meters, the JAKA SDK uses millimeters.
    pose.tran.x = msg.pose.position.x * 1000.0;
    pose.tran.y = msg.pose.position.y * 1000.0;
    pose.tran.z = msg.pose.position.z * 1000.0;

    pose.quat.s = msg.pose.orientation.w;
    pose.quat.x = msg.pose.orientation.x;
    pose.quat.y = msg.pose.orientation.y;
    pose.quat.z = msg.pose.orientation.z;
    Quaternion normalized;
    if (!jaka_teleop::quat_normalize(pose.quat, &normalized)) {
        return false;
    }
    pose.quat = normalized;
    return true;
}

// Per-cycle Cartesian servo limits. servo_p is fed one increment per 8 ms
// interpolation cycle.
constexpr double SERVO_TRAN_DEADBAND = 0.5;   // mm: ignore smaller translation deltas
constexpr double Y_LIMIT_MM = 110.0;         // workspace y boundary: left > -110, right < +110

// Drive one arm to track the controller's displacement since grip press, in
// Cartesian space. servo_p is sent one per-cycle INCR delta — the same
// feed-forward pattern as the working demo (servo_p(&cart, INCR) with a fixed
// increment). The grip-press pose is the origin: because INCR accumulates these
// increments, the arm ends up exactly at "origin + offset", returns to start
// when the controller returns to its origin, and holds under the continuous
// zero-increment stream after release. JAKA performs the Cartesian-to-joint IK.
//
// Rotation is relative to the controller orientation captured at Grip press.
// The bridge has already converted the controller quaternion to the robot base
// frame. We maintain the cumulative rotation actually sent to the robot, so a
// per-cycle speed clamp does not silently discard part of a fast hand rotation.
void servo_cartesian(JAKAZuRobot &robot,
                     const geometry_msgs::msg::PoseStamped &msg,
                     const CtrlPose &ctrl_origin,
                     CtrlPose &ctrl_prev,
                     const char *arm_name)
{
    refresh_governor_params();

    // Per-cycle command caps. The optional singularity term shrinks both the
    // translation and the rotation step when J5 approaches 0 or +-180 deg, where
    // the same Cartesian command costs a much larger joint motion.
    double step_cap = servo_max_tran_step;
    double rot_cap = rotation_max_step;
    if (servo_singularity_slowdown && actual_joints_valid &&
        steady_seconds() - actual_joints_time <= 0.3)
    {
        const double scale = singularity_scale();
        step_cap *= scale;
        rot_cap *= scale;
        static int sing_count = 0;
        if (scale < 0.999 && ++sing_count % 50 == 0)
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] near wrist singularity: J5 is %.1f deg from 0/180, "
                        "step scaled to %.0f%%", arm_name,
                        wrist_singularity_angle() * 180.0 / PI_D, scale * 100.0);
    }

    CtrlPose ctrl_now;
    if (!pose_to_ctrl(msg, ctrl_now))
    {
        if (debug)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] ignoring target with invalid orientation quaternion", arm_name);
        }
        return;
    }

    // Workspace y-limit: an absolute one-way barrier on the arm's TCP y so the
    // two arms never cross the shared center line. The commanded absolute
    // position is "TCP at grip + handle offset since grip"; we clamp the handle
    // target BEFORE computing the increment, so the arm stops exactly at the
    // boundary and is never commanded a reverse (pull-back) motion. No per-cycle
    // read-back, so there is no staleness and no bounce.
    if (enable_y_limit && arm_tcp_valid)
    {
        double pred_y = arm_tcp_y_at_grip + (ctrl_now.tran.y - ctrl_origin.tran.y);
        if (arm_is_right && pred_y > Y_LIMIT_MM)
            ctrl_now.tran.y = ctrl_origin.tran.y + (Y_LIMIT_MM - arm_tcp_y_at_grip);
        else if (!arm_is_right && pred_y < -Y_LIMIT_MM)
            ctrl_now.tran.y = ctrl_origin.tran.y + (-Y_LIMIT_MM - arm_tcp_y_at_grip);
    }

    // Leash: keep the hand displacement since the Grip press inside a sphere.
    // It is a relative displacement, so it needs no frame calibration and cannot
    // fight the y-limit; it only stops the operator from asking for a target the
    // arm cannot reach.
    if (servo_leash_radius_mm > 0.0)
    {
        double off_x = ctrl_now.tran.x - ctrl_origin.tran.x;
        double off_y = ctrl_now.tran.y - ctrl_origin.tran.y;
        double off_z = ctrl_now.tran.z - ctrl_origin.tran.z;
        if (jaka_guard::clamp_sphere(servo_leash_radius_mm, off_x, off_y, off_z))
        {
            ctrl_now.tran.x = ctrl_origin.tran.x + off_x;
            ctrl_now.tran.y = ctrl_origin.tran.y + off_y;
            ctrl_now.tran.z = ctrl_origin.tran.z + off_z;
            static int leash_count = 0;
            if (++leash_count % 50 == 0)
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] leash: hand offset clamped to %.0f mm",
                            arm_name, servo_leash_radius_mm);
        }
    }

    CartesianPose incr;
    // Per-cycle translation increment (vector subtraction is exact).
    incr.tran.x = ctrl_now.tran.x - ctrl_prev.tran.x;
    incr.tran.y = ctrl_now.tran.y - ctrl_prev.tran.y;
    incr.tran.z = ctrl_now.tran.z - ctrl_prev.tran.z;

    // Attitude control: controller rotation relative to the Grip-press origin.
    Quaternion rotation_step = jaka_teleop::quat_identity();
    Quaternion rotation_step_actual = jaka_teleop::quat_identity();
    bool has_rotation_step = false;
    incr.rpy.rx = 0.0;
    incr.rpy.ry = 0.0;
    incr.rpy.rz = 0.0;
    if (use_rpy_ctrl)
    {
        // Desired world-frame rotation since Grip press.
        Quaternion desired_rotation = jaka_teleop::quat_multiply(
            ctrl_now.quat, jaka_teleop::quat_conjugate(ctrl_origin.quat));
        desired_rotation = jaka_teleop::quat_normalized(desired_rotation);
        if (rotation_scale < 1.0)
        {
            const double desired_angle =
                jaka_teleop::quat_rotation_angle(desired_rotation);
            desired_rotation = jaka_teleop::quat_scaled_to_angle(
                desired_rotation, desired_angle * rotation_scale);
        }

        // Rotation which has not yet reached the robot.
        Quaternion remaining_rotation = jaka_teleop::quat_multiply(
            desired_rotation,
            jaka_teleop::quat_conjugate(commanded_rotation_delta));
        remaining_rotation = jaka_teleop::quat_normalized(remaining_rotation);
        remaining_rotation = jaka_teleop::quat_canonical(remaining_rotation);
        const double remaining_angle =
            jaka_teleop::quat_rotation_angle(remaining_rotation);

        if (remaining_angle >= rotation_deadband && rotation_gain > 0.0)
        {
            const double requested_step = remaining_angle * rotation_gain;
            const double applied_step = std::min(requested_step, rot_cap);
            rotation_step = jaka_teleop::quat_scaled_to_angle(
                remaining_rotation, applied_step);
            has_rotation_step = applied_step > 1e-6;
        }
    }

    if (has_rotation_step)
    {
        RotMatrix rot_incr;
        if (robot.quaternion_to_rot_matrix(&rotation_step, &rot_incr) != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] failed to convert quaternion to rot matrix", arm_name);
            incr.rpy.rx = 0.0;
            incr.rpy.ry = 0.0;
            incr.rpy.rz = 0.0;
            has_rotation_step = false;
        }
        else if (robot.rot_matrix_to_rpy(&rot_incr, &incr.rpy) != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] failed to convert rot matrix to RPY", arm_name);
            incr.rpy.rx = 0.0;
            incr.rpy.ry = 0.0;
            incr.rpy.rz = 0.0;
            has_rotation_step = false;
        }
        else
        {
            incr.rpy.rx *= rotation_sign;
            incr.rpy.ry *= rotation_sign;
            incr.rpy.rz *= rotation_sign;
            // Reconstruct the rotation represented by the exact signed RPY sent
            // to the controller. This keeps the accumulated reference correct
            // if the sign parameter is used or the Euler conversion changes axes.
            RotMatrix actual_rot;
            Quaternion actual_quat;
            if (robot.rpy_to_rot_matrix(&incr.rpy, &actual_rot) != 0 ||
                robot.rot_matrix_to_quaternion(&actual_rot, &actual_quat) != 0 ||
                !jaka_teleop::quat_normalize(actual_quat, &rotation_step_actual))
            {
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] failed to verify orientation increment", arm_name);
                incr.rpy.rx = 0.0;
                incr.rpy.ry = 0.0;
                incr.rpy.rz = 0.0;
                has_rotation_step = false;
            }
        }
    }

    // Deadband: hold in place instead of chasing sub-mm / sub-degree handle noise.
    double max_tran = std::max(std::fabs(incr.tran.x),
                      std::max(std::fabs(incr.tran.y), std::fabs(incr.tran.z)));
    if (max_tran < SERVO_TRAN_DEADBAND && !has_rotation_step)
        return;

    // Velocity clamp: scale translation to its per-cycle cap, preserving
    // direction. A fast handle swing moves the arm at the capped speed instead
    // of commanding an overspeed Cartesian move.
    if (max_tran > step_cap)
    {
        double s = step_cap / max_tran;
        incr.tran.x *= s;
        incr.tran.y *= s;
        incr.tran.z *= s;
    }

    // Acceleration limit: bound how fast the per-cycle increment may change, so
    // a sudden handle motion cannot ask the controller for a velocity step.
    jaka_guard::limit_accel(servo_trans_accel_mm_s2, SERVO_CYCLE_S, guard_accel,
                            incr.tran.x, incr.tran.y, incr.tran.z);

    // Commanded TCP = grip TCP + accumulated hand offset. Rotation happens about
    // the TCP, so it does not move this point.
    const double off_x = ctrl_now.tran.x - ctrl_origin.tran.x;
    const double off_y = ctrl_now.tran.y - ctrl_origin.tran.y;
    const double off_z = ctrl_now.tran.z - ctrl_origin.tran.z;
    const double cmd_x = arm_tcp_at_grip.tran.x + off_x;
    const double cmd_y = arm_tcp_at_grip.tran.y + off_y;
    const double cmd_z = arm_tcp_at_grip.tran.z + off_z;

    // Tracking gate: withhold the increment while the arm is measurably behind
    // the commanded point. ctrl_prev is deliberately NOT advanced, so the
    // pending motion collapses onto the current hand pose instead of queueing.
    bool held = false;
    double behind = -1.0;
    const double now_s = steady_seconds();
    // A decimated poll is still usable, so the staleness window follows the
    // measured step count instead of a fixed 50 ms.
    const double feedback_max_age_s = 0.02 + SERVO_CYCLE_S * feedback_stride * 3.0;
    if (servo_max_track_error_mm > 0.0 && actual_tcp_valid && arm_tcp_valid &&
        !track_gate_gave_up && now_s - actual_tcp_time <= feedback_max_age_s)
    {
        behind = jaka_guard::distance_mm(
            cmd_x, cmd_y, cmd_z, actual_tcp.tran.x, actual_tcp.tran.y, actual_tcp.tran.z);
        if (behind > servo_max_track_error_mm)
        {
            if (track_hold_since < 0.0)
                track_hold_since = now_s;
            if (now_s - track_hold_since > servo_track_deadman_s)
            {
                track_gate_gave_up = true;
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] tracking gate gave up: arm stayed %.1f mm behind for "
                            "%.1f s; release and press Grip to re-arm the gate",
                            arm_name, behind, servo_track_deadman_s);
            }
            else
            {
                held = true;
            }
        }
        else
        {
            track_hold_since = -1.0;
        }
    }
    else
    {
        track_hold_since = -1.0;
    }

    if (held)
    {
        servo_hold(robot);
        blackbox_line("servo", "track_hold", incr, off_x, off_y, off_z,
                      cmd_x, cmd_y, cmd_z, 0, true);
        static int hold_count = 0;
        if (++hold_count % 125 == 0)
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] holding: arm is %.1f mm behind the commanded point",
                        arm_name, behind);
        return;
    }

    if (!servo_timer_driven && !servo_send_allowed(now_s))
        return;  // the previous interpolation window is still open; retry next tick

    int ret = robot.servo_p(&incr, MoveMode::INCR);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] servo_p failed: %s", arm_name, mapErr[ret].c_str());
        blackbox_line("servo", "failed", incr, off_x, off_y, off_z,
                      cmd_x, cmd_y, cmd_z, ret, false);
        return;
    }
    guard_accel.x = incr.tran.x;
    guard_accel.y = incr.tran.y;
    guard_accel.z = incr.tran.z;
    blackbox_line("servo", "sent", incr, off_x, off_y, off_z,
                  cmd_x, cmd_y, cmd_z, ret, false);

    // Track what we actually commanded so the next cycle's delta is measured
    // from the same reference the robot is using.
    ctrl_prev = ctrl_now;
    if (has_rotation_step)
    {
        commanded_rotation_delta = jaka_teleop::quat_multiply(
            rotation_step_actual, commanded_rotation_delta);
        commanded_rotation_delta =
            jaka_teleop::quat_normalized(commanded_rotation_delta);
        commanded_rotation_delta =
            jaka_teleop::quat_canonical(commanded_rotation_delta);
    }

    // DEBUG: throttled print of the total translation offset and the per-cycle
    // incremental rotation (the quaternion-derived RPY delta).
    static int dbg_count = 0;
    if (++dbg_count % 25 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] offset=(%.1f,%.1f,%.1f)mm incr=(%.2f,%.2f,%.2f)mm rincr=(%.4f,%.4f,%.4f)rad",
                    arm_name,
                    ctrl_now.tran.x - ctrl_origin.tran.x,
                    ctrl_now.tran.y - ctrl_origin.tran.y,
                    ctrl_now.tran.z - ctrl_origin.tran.z,
                    incr.tran.x, incr.tran.y, incr.tran.z,
                    incr.rpy.rx, incr.rpy.ry, incr.rpy.rz);
    }
}

// The controller interpolates exactly one step_num*8 ms window per servo_p call.
// A second command that arrives before that window closes makes the controller
// report a "continuous servo command" warning and drop the step. The timer
// driven path cannot overlap by construction; this guard protects the message
// driven default against the back-to-back bursts the ROS callbacks produce.
bool servo_send_allowed(double now)
{
    if (now - last_servo_send < servo_min_period_s)
        return false;
    last_servo_send = now;
    return true;
}

bool servo_hold(JAKAZuRobot &robot)
{
    CartesianPose hold;
    hold.tran.x = 0.0;
    hold.tran.y = 0.0;
    hold.tran.z = 0.0;
    hold.rpy.rx = 0.0;
    hold.rpy.ry = 0.0;
    hold.rpy.rz = 0.0;
    int ret = robot.servo_p(&hold, MoveMode::INCR);
    if (ret != 0)
    {
        if (debug)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] servo_p HOLD failed: %s", arm_name.c_str(), mapErr[ret].c_str());
        }
        return false;
    }
    guard_accel.reset();
    return true;
}

// JAKA servo_p must be fed continuously. A one-shot zero increment is not a
// reliable brake on this controller, so keep publishing zero increments after
// Grip release. This is the equivalent of vr-teleop-kit continuing to send the
// last joint target while its clutch is disengaged.
void hold_stream_callback()
{
    // The SDK connection is single-owner: never poll it while the re-arm thread
    // is inside power_on/enable/servo_move_enable.
    if (!arm_ok || rearming)
        return;
    poll_servo_feedback();

    // Timer-driven servo: exactly one command per 8 ms window, whatever the ROS
    // message jitter is. The message-driven path is kept as a fallback.
    if (gripped && servo_timer_driven)
    {
        geometry_msgs::msg::PoseStamped pose;
        {
            std::lock_guard<std::mutex> lk(latest_mtx);
            pose = latest_pose;
        }
        servo_cartesian(robot, pose, ctrl_origin, ctrl_prev, arm_name.c_str());
        return;
    }
    if (gripped || !hold_stream_enabled)
        return;

    if (!servo_hold(robot))
    {
        ++hold_stream_failure_count;
        if (hold_stream_failure_count >= 5)
        {
            hold_stream_enabled = false;
            hold_stream_failure_count = 0;
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] stopping hold stream after repeated servo_p failures",
                        arm_name.c_str());
        }
    }
    else
    {
        hold_stream_failure_count = 0;
    }
}

// Re-arm the robot if protection dropped it out of servo. Called on every grip
// press so a re-grip recovers from a protective/collision stop. Checks the
// states top-down (servo -> enable -> power-on) and drives them bottom-up
// (power_on -> enable_robot -> servo_move_enable(true)), only performing the
// steps actually needed:
//   - already in servo -> nothing to do;
//   - collision / error  -> collision_recover() + clear_error();
//   - powered off        -> power_on (+8 s settle);
//   - disabled           -> enable_robot (+4 s settle);
//   - otherwise          -> just re-enter servo.
// Mirrors bring_up_arm(), but skips login and the steps that are still intact.
bool ensure_arm_ready(JAKAZuRobot &robot)
{
    if (!arm_ok)
        return false;

    // Fast path: still in servo, nothing to recover.
    BOOL in_servo = FALSE;
    if (robot.is_in_servomove(&in_servo) == 0 && in_servo)
        return true;

    // Clear a protective stop / collision / motion-abnormal error first, so the
    // controller will accept power_on/enable/servo again.
    BOOL in_collision = FALSE;
    if (robot.is_in_collision(&in_collision) == 0 && in_collision)
    {
        int r = robot.collision_recover();
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] collision_recover ret=%d", arm_name.c_str(), r);
    }
    int r = robot.clear_error();
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] clear_error ret=%d", arm_name.c_str(), r);

    // Read power-on / enabled flags. get_robot_status_simple is the lightweight
    // source for both (errcode / powered_on / enabled).
    RobotStatus_simple status{};
    if (robot.get_robot_status_simple(&status) != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] get_robot_status_simple failed; cannot re-arm", arm_name.c_str());
        return false;
    }

    if (status.powered_on == 0)
    {
        int ret = robot.power_on();
        if (ret != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] re-arm power_on FAILED (ret=%d: %s)", arm_name.c_str(),
                        ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "[%s] re-arm power_on OK", arm_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(8));
        // Power-on leaves the robot disabled; refresh the flags before deciding
        // whether enable_robot is still needed.
        if (robot.get_robot_status_simple(&status) != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] get_robot_status_simple failed after power_on", arm_name.c_str());
            return false;
        }
    }

    if (status.enabled == 0)
    {
        int ret = robot.enable_robot();
        if (ret != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] re-arm enable_robot FAILED (ret=%d: %s)", arm_name.c_str(),
                        ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "[%s] re-arm enable_robot OK", arm_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(4));
    }

    int ret = robot.servo_move_enable(true);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] re-arm servo_move_enable FAILED (ret=%d: %s)", arm_name.c_str(),
                    ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "[%s] re-arm servo enabled", arm_name.c_str());
    return true;
}

// Re-arm the robot in a background thread. The slow ensure_arm_ready sequence
// (power_on/enable/servo with multi-second settles) must NOT run in the executor
// thread, or it freezes target_callback for 12+ s. It also must NOT run
// concurrently with servo_p — the JAKA SDK holds one connection per process — so
// target_callback stays idle (gripped false) until this finishes.
void rearm_worker()
{
    bool ok = ensure_arm_ready(robot);
    if (ok && grip_now)
    {
        CtrlPose origin;
        {
            std::lock_guard<std::mutex> lk(latest_mtx);
            if (!pose_to_ctrl(latest_pose, origin))
            {
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] cannot start tracking without a valid controller orientation",
                            arm_name.c_str());
                rearming = false;
                return;
            }
        }
        // Capture the arm's absolute TCP y once at grip press so the workspace
        // y-limit is enforced against the actual position (grip TCP + handle
        // offset) without a stale per-cycle read-back.
        CartesianPose tcp_at_grip;
        arm_tcp_valid = (robot.get_tcp_position(&tcp_at_grip) == 0);
        if (arm_tcp_valid)
        {
            arm_tcp_y_at_grip = tcp_at_grip.tran.y;
            arm_tcp_at_grip = tcp_at_grip;
        }
        // Every Grip press starts a clean governor epoch: no inherited velocity
        // and a fresh chance for the tracking gate.
        guard_accel.reset();
        track_hold_since = -1.0;
        track_gate_gave_up = false;
        // Write origin/prev BEFORE publishing gripped=true so the atomic gate
        // makes these writes visible to target_callback.
        ctrl_origin = origin;
        ctrl_prev = origin;
        commanded_rotation_delta = jaka_teleop::quat_identity();
        gripped = true;
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] gripped, ctrl_origin=(%.1f,%.1f,%.1f)mm, 6D=%s",
                    arm_name.c_str(),
                    ctrl_origin.tran.x, ctrl_origin.tran.y, ctrl_origin.tran.z,
                    use_rpy_ctrl ? "true" : "false");
    }
    rearming = false;
}

// Update the grip state for one arm. Separate press/release thresholds form a
// Schmitt trigger, preventing a noisy grip value near 0.5 from repeatedly
// engaging and releasing the arm.
void handle_grip(float value)
{
    const bool was_pressed = grip_now.load();
    const bool now = jaka_teleop::grip_state(
        was_pressed, value, grip_press_threshold, grip_release_threshold);
    grip_now = now;  // mirror current grip so the re-arm thread can check it
    if (now != was_pressed)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] Grip %s (raw=%.3f, press>%.2f release<%.2f)",
                    arm_name.c_str(), now ? "pressed" : "released", value,
                    grip_press_threshold, grip_release_threshold);
    }
    if (now)
        hold_stream_enabled = false;
    if (now && !gripped && !rearming)
    {
        // Reap a previous, already-finished re-arm thread before overwriting the
        // std::thread (assigning to a joinable std::thread would terminate).
        if (rearm_thread.joinable())
            rearm_thread.join();
        rearming = true;
        rearm_thread = std::thread(rearm_worker);
    }
    else if (!now)
    {
        gripped = false;
        // Let the 8 ms timer send the first zero increment. Calling servo_hold
        // here as well lands inside the window of the previous command often
        // enough that the controller reports a continuous-servo-command warning.
        hold_stream_enabled = true;
    }
}

void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!arm_ok)
        return;
    {
        std::lock_guard<std::mutex> lk(latest_mtx);
        latest_pose = *msg;
    }
    if (gripped && !servo_timer_driven)
    {
        servo_cartesian(robot, *msg, ctrl_origin, ctrl_prev, arm_name.c_str());
    }
    // Not gripped: the independent 125 Hz timer keeps sending zero increments.
    if(debug)
    {
        RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                    "[%s] target_callback: %f,%f,%f",
                    arm_name.c_str(), msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    }
}

void button_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    float grip = msg->axes.size() > static_cast<size_t>(grip_axis)
                     ? msg->axes[grip_axis] : 0.0f;
    if (arm_ok)
        handle_grip(grip);

    if (gripper != nullptr)
    {
        // Two gripper control modes:
        // - "digital": binary control via a Joy button -> set_closed(bool)
        // - "linear" : analog control via a Joy axis -> Grip(float)
        if (!gripper_linear)
        {
            bool pressed = msg->buttons.size() > static_cast<size_t>(gripper_button_index) &&
                           msg->buttons[gripper_button_index] != 0;
            if (pressed != gripper->closed())
            {
                std::string error;
                if (!gripper->set_closed(pressed, &error))
                {
                    RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper command failed: %s",
                                arm_name.c_str(), error.c_str());
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper %s",
                                arm_name.c_str(), pressed ? "closed" : "opened");
                }
            }
        }
        else // linear (analog) control
        {
            float trigger_raw = msg->axes.size() > static_cast<size_t>(gripper_axis)
                                  ? msg->axes[gripper_axis] : 0.0f;
            // Normalize/clamp to [0,1]. Many controllers already report 0..1.
            float trigger = trigger_raw;
            if (trigger < 0.0f) trigger = 0.0f;
            if (trigger > 1.0f) trigger = 1.0f;
            if (std::fabs(trigger - prev_gripper_trigger) >= gripper_trigger_deadband)
            {
                std::string error;
                if (!gripper->Grip(trigger, &error))
                {
                    RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper Grip(%.3f) failed: %s",
                                arm_name.c_str(), trigger, error.c_str());
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                                "[%s] PGI gripper Grip(%.3f)",
                                arm_name.c_str(), trigger);
                }
                prev_gripper_trigger = trigger;
            }
        }
    }
}

// Bring one arm fully online: login -> configure TIO -> power_on -> enable ->
// servo. Mirrors the jaka_driver reference sequence (wait 8 s after power_on
// and 4 s after enable so the robot reaches the powered/enabled state before
// servo). Returns true only if every step succeeded; on failure the arm is
// left un-driven and the callbacks skip it.
bool bring_up_arm(JAKAZuRobot &robot, const std::string &ip, const char *arm)
{
    int ret = robot.login_in(ip.c_str());
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s login FAILED (%s, ret=%d: %s)",
                    arm, ip.c_str(), ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s login OK (%s)", arm, ip.c_str());

    // Same SDK setup the reference node applies right after login.
    robot.set_status_data_update_time_interval(100);
    robot.set_block_wait_timeout(120);

    // Configure the TIO while the controller is logged in but before the arm
    // enters servo mode. JAKA controllers may reject pin-mode changes once
    // servo motion is active.
    if (gripper != nullptr)
    {
        std::string error;
        if (!gripper->initialize(&error))
        {
            RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                        "[%s] PGI gripper disabled: %s", arm_name.c_str(), error.c_str());
            gripper.reset();
        }
        else
        {
            const auto & config = gripper->config();
            RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                        "[%s] PGI-140-80 ready: RS485 channel=%d slave=%d force=%d%% speed=%d%% "
                        "open=%d closed=%d trigger_axis=%d mode=%s button=%d deadband=%.3f",
                        arm_name.c_str(), config.rs485_channel, config.slave_id,
                        config.force_percent, config.speed_percent,
                        config.open_position, config.closed_position,
                        gripper_axis, (gripper_linear ? "linear" : "digital"),
                        (gripper_linear ? -1 : gripper_button_index),
                        gripper_trigger_deadband);
        }
    }

    ret = robot.power_on();
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s power_on FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s power_on OK", arm);
    std::this_thread::sleep_for(std::chrono::seconds(8));

    ret = robot.enable_robot();
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s enable_robot FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s enable_robot OK", arm);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    // Keep collision detection ON but raise the threshold 25N -> 75N. Level 0
    // (disable) would let the arm push through obstacles — unsafe in VR teleop.
    robot.set_collision_level(3);

    // Hard Cartesian velocity/acceleration/jerk limits for servo_p (mm/s,
    // mm/s^2, mm/s^3) plus orientation limits (deg/s, deg/s^2, deg/s^3):
    // a safety bound on top of the per-cycle software clamp.

    //滤波设置
    //非线性滤波
    ret = robot.servo_move_use_carte_NLF(1500.0, 2500.0, 2800.0,1120.0, 1180.0, 1720.0);
    //速度前瞻滤波
    //ret = robot.servo_speed_foresight(10, 0.5);
    if(ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s servo_speed_foresight FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }


    ret = robot.servo_move_enable(true);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"), "%s servo_move_enable FAILED (ret=%d: %s)",
                    arm, ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return false;
    }
    BOOL in_servo = FALSE;
    robot.is_in_servomove(&in_servo);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"), "%s servo enabled (is_in_servomove=%d)", arm, in_servo);
    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "C");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rc_ctrl_node");
    g_node = node;

    // Which arm this process controls. The JAKA SDK limits us to one connection
    // per process, so one instance drives exactly one arm and the launch file
    // starts two instances (arm_name left / right).
    arm_name = node->declare_parameter<string>("arm_name", "left");
    bool is_right = (arm_name == "right");
    arm_is_right = is_right;
    string default_ip = is_right ? "192.168.71.36" : "192.168.71.37";
    string default_topic = is_right ? "/rc_ctrl/right_target" : "/rc_ctrl/left_target";
    int default_grip_axis = is_right ? 1 : 0;

    // Read connection/topic parameters (defaults match RC_ctrl.launch.py).
    string ip = node->declare_parameter<string>("robot_ip", default_ip);
    string target_topic = node->declare_parameter<string>("target_topic", default_topic);
    string button_topic = node->declare_parameter<string>("button_topic", "/rc_ctrl/button");
    grip_axis = node->declare_parameter<int>("grip_axis", default_grip_axis);

    // PGI-140-80 is driven through the Quest index-trigger axis by default:
    // Joy axes[2] = leftTrig and axes[3] = rightTrig. A Joy button index can
    // be selected for controllers whose front button is reported digitally.
    gripper_axis = node->declare_parameter<int>(
        "gripper_axis", is_right ? 3 : 2);
    gripper_use_button = node->declare_parameter<bool>("gripper_use_button", false);
    const int configured_button_index = node->declare_parameter<int>(
        "gripper_button_index", -1);
    // Joy.buttons follows quest_reader's stable order; LTr=10 and RTr=11.
    gripper_button_index = configured_button_index >= 0 ? configured_button_index :
        (is_right ? 11 : 10);
    gripper_linear = node->declare_parameter<bool>("gripper_linear", true);
    gripper_trigger_deadband = node->declare_parameter<double>("gripper_trigger_deadband", 0.02);
    // Backwards compatibility: if the old boolean is set, prefer digital mode.
    if (gripper_use_button) {
        RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                    "[%s] parameter gripper_use_button is present; using digital mode",
                    arm_name.c_str());
        gripper_linear = false;
    }
    // gripper_trigger_threshold removed; using deadband for linear control

    pgi140::GripperConfig gripper_config;
    gripper_config.rs485_channel = node->declare_parameter<int>("gripper_rs485_channel", 1);
    gripper_config.slave_id = node->declare_parameter<int>("gripper_slave_id", 1);
    gripper_config.baudrate = node->declare_parameter<int>("gripper_baudrate", 115200);
    gripper_config.force_percent = node->declare_parameter<int>("gripper_force", 20);
    gripper_config.speed_percent = node->declare_parameter<int>("gripper_speed", 100);
    gripper_config.open_position = node->declare_parameter<int>("gripper_open_position", 1000);
    gripper_config.closed_position = node->declare_parameter<int>(
        "gripper_closed_position", 0);
    gripper_config.initialize = node->declare_parameter<bool>("gripper_initialize", true);
    gripper_config.initialize_command = node->declare_parameter<int>(
        "gripper_initialize_command", 0x01);
    gripper_config.initialize_delay_ms = node->declare_parameter<int>(
        "gripper_initialize_delay_ms", 2000);
    gripper_config.configure_tio = node->declare_parameter<bool>(
        "gripper_configure_tio", false);
    gripper_config.enable_tio_power = node->declare_parameter<bool>(
        "gripper_enable_tio_power", true);
    gripper_config.tio_voltage = node->declare_parameter<int>(
        "gripper_tio_voltage", 0);

    
   debug = node->declare_parameter<bool>("debug", false);
   use_rpy_ctrl = node->declare_parameter<bool>("use_rpy_ctrl", true);
   rotation_scale = node->declare_parameter<double>("rotation_scale", 0.5);
   rotation_gain = node->declare_parameter<double>("rotation_gain", 0.5);
   rotation_deadband = node->declare_parameter<double>(
       "rotation_deadband_rad", 0.005);
   rotation_max_step = node->declare_parameter<double>(
       "rotation_max_step_rad", 0.0015);
   rotation_sign = node->declare_parameter<int>("rotation_sign", 1);
   grip_press_threshold = node->declare_parameter<double>(
       "grip_press_threshold", 0.6);
   grip_release_threshold = node->declare_parameter<double>(
       "grip_release_threshold", 0.4);
   enable_y_limit = node->declare_parameter<bool>("enable_y_limit", true);
   servo_max_tran_step = node->declare_parameter<double>(
       "servo_max_tran_step_mm", 1.0);
   if (!std::isfinite(servo_max_tran_step) || servo_max_tran_step <= 0.0 ||
       servo_max_tran_step > 5.0)
   {
       RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                   "[%s] servo_max_tran_step_mm must be in (0, 5]; using 1.0",
                   arm_name.c_str());
       servo_max_tran_step = 1.0;
   }

   // Conservative governor limits. All are live-tunable with `ros2 param set`.
   servo_leash_radius_mm = node->declare_parameter<double>(
       "servo_leash_radius_mm", 600.0);
   servo_trans_accel_mm_s2 = node->declare_parameter<double>(
       "servo_trans_accel_mm_s2", 1500.0);
   servo_max_track_error_mm = node->declare_parameter<double>(
       "servo_max_track_error_mm", 10.0);
   servo_track_deadman_s = node->declare_parameter<double>(
       "servo_track_deadman_s", 2.0);
   servo_feedback_enabled = node->declare_parameter<bool>(
       "servo_feedback_enabled", true);
   servo_feedback_budget_us = node->declare_parameter<double>(
       "servo_feedback_budget_us", 200.0);
   servo_singularity_slowdown = node->declare_parameter<bool>(
       "servo_singularity_slowdown", false);
   servo_singularity_threshold_deg = node->declare_parameter<double>(
       "servo_singularity_threshold_deg", 15.0);
   servo_singularity_min_scale = node->declare_parameter<double>(
       "servo_singularity_min_scale", 0.35);
   servo_timer_driven = node->declare_parameter<bool>(
       "servo_timer_driven", false);
   servo_min_period_s = node->declare_parameter<double>(
       "servo_min_period_ms", 8.5) / 1000.0;
   const std::string blackbox_dir = node->declare_parameter<std::string>(
       "servo_blackbox_dir", std::string("log/teleop"));
   if (!std::isfinite(servo_leash_radius_mm) || servo_leash_radius_mm < 0.0)
       servo_leash_radius_mm = 600.0;
   if (!std::isfinite(servo_trans_accel_mm_s2) || servo_trans_accel_mm_s2 < 0.0)
       servo_trans_accel_mm_s2 = 1500.0;
   if (!std::isfinite(servo_max_track_error_mm) || servo_max_track_error_mm < 0.0)
       servo_max_track_error_mm = 10.0;
   if (!std::isfinite(servo_track_deadman_s) || servo_track_deadman_s < 0.0)
       servo_track_deadman_s = 2.0;
   if (!std::isfinite(servo_feedback_budget_us) || servo_feedback_budget_us < 50.0)
       servo_feedback_budget_us = 200.0;
   if (!std::isfinite(servo_singularity_threshold_deg) ||
       servo_singularity_threshold_deg < 0.0 || servo_singularity_threshold_deg >= 90.0)
       servo_singularity_threshold_deg = 15.0;
   if (!std::isfinite(servo_singularity_min_scale) ||
       servo_singularity_min_scale <= 0.0 || servo_singularity_min_scale > 1.0)
       servo_singularity_min_scale = 0.35;

   if (!blackbox_dir.empty())
   {
       std::error_code ec;
       std::filesystem::create_directories(blackbox_dir, ec);
       const auto stamp = std::to_string(std::chrono::duration_cast<
           std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count());
       const std::string path = blackbox_dir + "/" + arm_name + "_" + stamp + ".csv";
       blackbox.open(path, std::ios::out | std::ios::trunc);
       if (blackbox.is_open())
       {
           blackbox_header();
           blackbox.flush();
           RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                       "[%s] black box: %s", arm_name.c_str(), path.c_str());
       }
       else
       {
           RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                       "[%s] cannot open black box '%s'; continuing without it",
                       arm_name.c_str(), path.c_str());
       }
   }

   rotation_scale = std::max(0.0, std::min(1.0, rotation_scale));
   rotation_gain = std::max(0.0, std::min(1.0, rotation_gain));
   rotation_deadband = std::max(0.0, rotation_deadband);
   if (rotation_max_step <= 0.0)
   {
       RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                   "[%s] rotation_max_step_rad must be positive; using 0.0015",
                   arm_name.c_str());
       rotation_max_step = 0.0015;
   }
   if (rotation_sign != 1 && rotation_sign != -1)
   {
       RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                   "[%s] rotation_sign must be 1 or -1; using 1",
                   arm_name.c_str());
       rotation_sign = 1;
   }
   grip_press_threshold = std::max(0.0, std::min(1.0, grip_press_threshold));
   grip_release_threshold = std::max(0.0, std::min(1.0, grip_release_threshold));
   if (grip_press_threshold <= grip_release_threshold)
   {
       RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                   "[%s] grip_press_threshold must exceed grip_release_threshold; "
                   "using 0.6/0.4", arm_name.c_str());
       grip_press_threshold = 0.6;
       grip_release_threshold = 0.4;
   }


    gripper = std::make_unique<pgi140::Pgi140Gripper>(robot, gripper_config);
    arm_ok = bring_up_arm(robot, ip, arm_name.c_str());
    if (!arm_ok)
    {
        gripper.reset();
    }
    if (arm_ok)
    {
        calibrate_servo_feedback();
        log_robot_configuration();
    }

    // Subscribe to the handle target topic and the shared button topic.
    // keep_last(1): teleop 控制循环只关心最新一帧，depth=10 会在消费端稍慢时
    // 积压旧位姿、引入递增延迟。与 quest_reader 端 QoSProfile(depth=1) 兼容。
    auto target_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic, rclcpp::QoS(1), target_callback);
    auto button_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        button_topic, rclcpp::QoS(1), button_callback);
    auto hold_timer = node->create_wall_timer(
        std::chrono::milliseconds(8), hold_stream_callback);
    (void)hold_timer;

    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "rc_ctrl_node started: arm=%s, ip=%s, tracking %s (grip axis %d)",
                arm_name.c_str(), ip.c_str(), target_topic.c_str(), grip_axis);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] orientation control: enabled=%s scale=%.2f gain=%.2f "
                "deadband=%.4f rad max_step=%.4f rad sign=%d",
                arm_name.c_str(), use_rpy_ctrl ? "true" : "false",
                rotation_scale, rotation_gain, rotation_deadband,
                rotation_max_step, rotation_sign);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] grip hysteresis: press>%.2f release<%.2f; "
                "continuous 125 Hz hold when disengaged",
                arm_name.c_str(), grip_press_threshold, grip_release_threshold);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] servo translation cap: %.2f mm/8 ms cycle (%.0f mm/s)",
                arm_name.c_str(), servo_max_tran_step, servo_max_tran_step * 125.0);
    RCLCPP_INFO(rclcpp::get_logger("rc_ctrl"),
                "[%s] governors: leash=%.0fmm accel=%.0fmm/s2 track_gate=%.1fmm "
                "deadman=%.1fs singularity=%s(%.0fdeg,min %.2f) timer_driven=%s "
                "min_period=%.1fms blackbox=%s",
                arm_name.c_str(), servo_leash_radius_mm, servo_trans_accel_mm_s2,
                servo_max_track_error_mm, servo_track_deadman_s,
                servo_singularity_slowdown ? "on" : "off",
                servo_singularity_threshold_deg, servo_singularity_min_scale,
                servo_timer_driven ? "on" : "off", servo_min_period_s * 1000.0,
                blackbox.is_open() ? "on" : "off");

    rclcpp::spin(node);

    // Wait for an in-flight background re-arm to finish before touching the SDK
    // below (an active re-arm can block up to ~12 s on its power_on/enable
    // settles). Joining first avoids servo_move_enable/login_out racing it.
    if (rearm_thread.joinable())
        rearm_thread.join();

    // Graceful shutdown. Only touch the arm if it actually connected: calling
    // servo_move_enable/login_out on a disconnected arm blocks until the SDK
    // gives up, which hangs Ctrl-C and forces a SIGKILL — and a SIGKILL leaves
    // a stale session on the controller, blocking the next login.
    if (arm_ok)
    {
        if (gripper != nullptr && gripper->closed())
        {
            std::string error;
            if (!gripper->set_closed(false, &error))
            {
                RCLCPP_WARN(rclcpp::get_logger("rc_ctrl"),
                            "[%s] failed to open PGI gripper on shutdown: %s",
                            arm_name.c_str(), error.c_str());
            }
        }
        robot.servo_move_enable(false);
        robot.login_out();
    }
    if (blackbox.is_open())
    {
        blackbox.flush();
        blackbox.close();
    }
    rclcpp::shutdown();
    return 0;
}
