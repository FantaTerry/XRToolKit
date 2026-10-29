#ifndef JAKA_DRIVER_TELEOP_CONTROL_HPP_
#define JAKA_DRIVER_TELEOP_CONTROL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "jaka_driver/orientation_math.hpp"

namespace jaka_teleop
{

// No ROS, connection, clock or SDK calls: the one SDK owner supplies monotonic
// seconds and commits exactly one decision after servo_p returns.
struct ControllerPose
{
  CartesianTran tran{};                 // mapped controller coordinates, mm
  Quaternion quat = quat_identity();
};

struct Config
{
  double period_s = 0.008;               // MUST equal servo step_num * 8 ms
  double max_interval_s = 0.032;
  double input_timeout_s = 0.2;
  double feedback_timeout_s = 0.3;
  double translation_speed_mm_s = 125.0; // per-axis command limits
  double translation_accel_mm_s2 = 500.0;
  double translation_deadband_mm = 0.5;
  double rotation_speed_rad_s = 0.1875;  // angular velocity vector norm
  double rotation_accel_rad_s2 = 0.75;
  double rotation_deadband_rad = 0.005;
  double rotation_scale = 0.5;
  double rotation_gain = 0.5;            // normalized to an 8 ms cycle
  double rotation_max_step_rad = 0.0015;
  int rotation_sign = 1;
  bool use_rotation = true;
  bool enable_y_limit = true;
  bool arm_is_right = false;
  double y_limit_mm = 110.0;
};

enum class State {Hold, Tracking, AwaitRelease, Fault};

inline const char * state_name(State state)
{
  switch (state) {
    case State::Hold: return "hold";
    case State::Tracking: return "tracking";
    case State::AwaitRelease: return "await_release";
    case State::Fault: return "fault";
  }
  return "fault";
}

// R = Rz(rz) * Ry(ry) * Rx(rx). This explicitly preserves the existing
// world-fixed, left-composed small-rotation assumption. The vendor must confirm
// that servo_p(INCR) uses this convention in the installed controller version.
inline Quaternion quat_from_rpy(const Rpy & r)
{
  const double cx = std::cos(r.rx / 2), sx = std::sin(r.rx / 2);
  const double cy = std::cos(r.ry / 2), sy = std::sin(r.ry / 2);
  const double cz = std::cos(r.rz / 2), sz = std::sin(r.rz / 2);
  return quat_normalized({cx*cy*cz + sx*sy*sz, sx*cy*cz - cx*sy*sz,
    cx*sy*cz + sx*cy*sz, cx*cy*sz - sx*sy*cz});
}

inline Rpy rpy_from_quat(const Quaternion & input)
{
  const Quaternion q = quat_normalized(input);
  Rpy r{};
  r.rx = std::atan2(2*(q.s*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y));
  r.ry = std::asin(std::max(-1.0, std::min(1.0, 2*(q.s*q.y - q.z*q.x))));
  r.rz = std::atan2(2*(q.s*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  return r;
}

using Vector3 = std::array<double, 3>;
inline double vector_norm(const Vector3 & v)
{
  return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
inline Vector3 rotation_vector(const Quaternion & input)
{
  const Quaternion q = quat_canonical(quat_normalized(input));
  const double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-12) {return {{0, 0, 0}};}
  const double angle = 2 * std::atan2(n, std::max(0.0, q.s));
  return {{q.x * angle / n, q.y * angle / n, q.z * angle / n}};
}
inline Quaternion quat_from_rotation_vector(const Vector3 & v)
{
  const double angle = vector_norm(v);
  if (angle < 1e-12) {return quat_identity();}
  const double scale = std::sin(angle / 2) / angle;
  return {std::cos(angle / 2), v[0] * scale, v[1] * scale, v[2] * scale};
}

struct Decision
{
  bool send = false;
  CartesianPose increment{};          // newly constructed INCR, never replayed
  CartesianTran raw_translation{};   // unclamped mapped controller displacement
  CartesianTran target_translation{};// absolute target in feedback TCP frame
  State state = State::Hold;
  std::string reason;
};

class TeleopControl
{
public:
  explicit TeleopControl(const Config & config = Config()) : config_(config)
  {
    const double positive[] = {config.period_s, config.max_interval_s,
      config.input_timeout_s, config.feedback_timeout_s,
      config.translation_speed_mm_s, config.translation_accel_mm_s2,
      config.rotation_speed_rad_s, config.rotation_accel_rad_s2,
      config.rotation_max_step_rad, config.y_limit_mm};
    for (double value : positive) {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("teleop timing and motion limits must be finite and positive");
      }
    }
    if (config.max_interval_s < config.period_s ||
      !std::isfinite(config.translation_deadband_mm) || config.translation_deadband_mm < 0 ||
      !std::isfinite(config.rotation_deadband_rad) || config.rotation_deadband_rad < 0 ||
      !std::isfinite(config.rotation_scale) || config.rotation_scale < 0 || config.rotation_scale > 1 ||
      !std::isfinite(config.rotation_gain) || config.rotation_gain < 0 || config.rotation_gain > 1 ||
      (config.rotation_sign != 1 && config.rotation_sign != -1)) {
      throw std::invalid_argument("invalid teleop deadband, rotation or watchdog configuration");
    }
  }

  void set_pose(const ControllerPose & pose, double received, bool tracked = true)
  {
    if (!std::isfinite(received) || received < 0 || received < pose_time_) {return;}
    pose_time_ = received;
    latest_input_ = pose;
    pose_valid_ = tracked && finite(pose.tran) &&
      quat_normalize(pose.quat, &latest_input_.quat);
    if (!pose_valid_ && state_ == State::Tracking) {
      external_pause("tracking_lost_or_invalid_pose");
    }
  }

  void set_grip(bool pressed, double received)
  {
    if (!std::isfinite(received) || received < 0 || received <= grip_time_) {return;}
    grip_time_ = received;
    grip_ = pressed;
    // Process release when observed, even if a later press arrives before plan.
    if (!pressed && state_ != State::Fault) {
      clear_residual();
      transition(State::Hold, "grip_released");
    }
  }

  void set_feedback(const CartesianPose & pose, double received, bool valid = true)
  {
    if (!std::isfinite(received) || received < 0 || received < feedback_time_) {return;}
    feedback_time_ = received;
    feedback_ = pose;
    feedback_valid_ = valid && finite(pose.tran) &&
      std::isfinite(pose.rpy.rx) && std::isfinite(pose.rpy.ry) && std::isfinite(pose.rpy.rz);
  }

  Decision plan(double now)
  {
    Decision decision;
    decision.state = state_;
    decision.reason = reason_;
    if (state_ == State::Fault || pending_ || !std::isfinite(now)) {return decision;}
    if (std::isfinite(last_plan_time_) && now - last_plan_time_ < config_.period_s - 1e-9) {
      return decision;
    }
    const bool overrun = std::isfinite(last_plan_time_) &&
      now - last_plan_time_ > config_.max_interval_s;
    last_plan_time_ = now; // never replay missed cycles
    if (overrun) {external_pause("send_interval_exceeded");}
    const bool fresh_input = pose_valid_ && fresh(now, pose_time_, config_.input_timeout_s) &&
      fresh(now, grip_time_, config_.input_timeout_s);
    const bool fresh_feedback = feedback_valid_ &&
      fresh(now, feedback_time_, config_.feedback_timeout_s);
    if (grip_ && !fresh_input) {external_pause("input_expired_or_untracked");}
    if (state_ == State::Tracking && !fresh_feedback) {
      external_pause("feedback_expired");
    }
    if (state_ == State::Hold && grip_ && fresh_input) {
      if (fresh_feedback) {engage();}
      else {external_pause("engagement_requires_valid_feedback");}
    }
    decision.send = true; // HOLD always creates a fresh all-zero value
    if (state_ == State::Tracking) {plan_tracking(decision);}
    decision.target_translation = target_.tran;
    decision.state = state_;
    decision.reason = reason_;
    pending_ = true;
    pending_decision_ = decision;
    return decision;
  }

  void commit(int sdk_return, double ended)
  {
    if (!pending_) {return;}
    pending_ = false;
    if (sdk_return != 0) {
      fault(sdk_return, ended, "servo_p");
      return;
    }
    // Success means submission, not proof that the robot reached this pose.
    if (pending_decision_.state == State::Tracking && state_ == State::Tracking) {
      submitted_ = planned_;
      submitted_rotation_ = planned_rotation_;
      velocity_ = planned_velocity_;
      angular_velocity_ = planned_angular_velocity_;
    }
    if (!std::isfinite(ended) || ended - last_plan_time_ > config_.max_interval_s) {
      external_pause("servo_call_exceeded_deadline");
    }
  }

  void external_pause(const std::string & reason)
  {
    if (state_ == State::Fault) {return;}
    clear_residual();
    transition(State::AwaitRelease, reason);
  }

  void fault(int code, double now, const std::string & context)
  {
    if (state_ == State::Fault) {return;}
    fault_code_ = code;
    fault_time_ = now;
    clear_residual();
    pending_ = false;
    transition(State::Fault, context);
  }

  const Config & config() const {return config_;}
  State state() const {return state_;}
  const std::string & reason() const {return reason_;}
  const CartesianPose & submitted() const {return submitted_;}
  const CartesianPose & target() const {return target_;}
  const CartesianPose & planned() const {return planned_;}
  const CartesianPose & feedback() const {return feedback_;}
  const ControllerPose & latest_input() const {return latest_input_;}
  int fault_code() const {return fault_code_;}
  double fault_time() const {return fault_time_;}
  const Vector3 & velocity() const {return velocity_;}
  const Vector3 & angular_velocity() const {return angular_velocity_;}

private:
  static bool finite(const CartesianTran & v)
  {return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);}
  static bool fresh(double now, double received, double timeout)
  {return std::isfinite(received) && now >= received && now - received <= timeout;}
  static double clamp(double value, double lower, double upper)
  {return std::max(lower, std::min(upper, value));}
  void transition(State next, const std::string & reason)
  {state_ = next; reason_ = reason;}
  void clear_residual()
  {
    target_ = submitted_;
    planned_ = submitted_;
    target_rotation_ = submitted_rotation_;
    planned_rotation_ = submitted_rotation_;
    velocity_ = planned_velocity_ = {{0, 0, 0}};
    angular_velocity_ = planned_angular_velocity_ = {{0, 0, 0}};
    accepted_offset_ = {{0, 0, 0}};
  }
  void engage()
  {
    controller_origin_ = latest_input_;
    robot_origin_ = feedback_;
    submitted_ = feedback_; // a new clutch reference must come from feedback
    origin_rotation_ = quat_from_rpy(robot_origin_.rpy);
    submitted_rotation_ = origin_rotation_;
    clear_residual();
    // An out-of-range engagement must not command an unsolicited return; allow
    // motion back in, but prohibit any further outward displacement.
    y_barrier_ = config_.arm_is_right ? std::max(config_.y_limit_mm, robot_origin_.tran.y) :
      std::min(-config_.y_limit_mm, robot_origin_.tran.y);
    transition(State::Tracking, "fresh_grip_reference");
  }

  // Distance required by a sampled velocity u and maximal subsequent braking.
  // This avoids endpoint snap violating the acceleration bound for a fixed
  // target. A moving/reversing target can be overshot while braking, by design.
  static double stopping_distance(double u, double acceleration, double dt)
  {
    const double reduction = acceleration * dt;
    const double count = std::ceil(u / reduction);
    return dt * std::max(0.0, count * u - reduction * count * (count - 1) / 2);
  }
  static double speed_toward(double error, double max_speed, double acceleration, double dt)
  {
    double low = 0, high = std::min(max_speed, std::abs(error) / dt);
    for (int i = 0; i < 36; ++i) {
      const double middle = (low + high) / 2;
      if (stopping_distance(middle, acceleration, dt) <= std::abs(error)) {low = middle;}
      else {high = middle;}
    }
    return std::copysign(low, error);
  }

  void plan_tracking(Decision & decision)
  {
    const double dt = config_.period_s;
    const Vector3 raw{{latest_input_.tran.x - controller_origin_.tran.x,
      latest_input_.tran.y - controller_origin_.tran.y,
      latest_input_.tran.z - controller_origin_.tran.z}};
    decision.raw_translation = {raw[0], raw[1], raw[2]};
    const double noise = std::max(std::abs(raw[0] - accepted_offset_[0]),
      std::max(std::abs(raw[1] - accepted_offset_[1]), std::abs(raw[2] - accepted_offset_[2])));
    // Translation noise is gated independently of rotation. The target freezes;
    // an already accepted residual may finish, then the stream is exactly zero.
    if (noise >= config_.translation_deadband_mm) {accepted_offset_ = raw;}
    target_.tran = {robot_origin_.tran.x + accepted_offset_[0],
      robot_origin_.tran.y + accepted_offset_[1], robot_origin_.tran.z + accepted_offset_[2]};
    if (config_.enable_y_limit) {
      target_.tran.y = config_.arm_is_right ? std::min(y_barrier_, target_.tran.y) :
        std::max(y_barrier_, target_.tran.y);
    }
    const Vector3 error{{target_.tran.x - submitted_.tran.x,
      target_.tran.y - submitted_.tran.y, target_.tran.z - submitted_.tran.z}};
    Vector3 delta{};
    for (size_t i = 0; i < 3; ++i) {
      const double desired = speed_toward(error[i], config_.translation_speed_mm_s,
        config_.translation_accel_mm_s2, dt);
      planned_velocity_[i] = velocity_[i] + clamp(desired - velocity_[i],
        -config_.translation_accel_mm_s2 * dt, config_.translation_accel_mm_s2 * dt);
      delta[i] = planned_velocity_[i] * dt;
    }
    planned_ = submitted_;
    planned_.tran = {submitted_.tran.x + delta[0], submitted_.tran.y + delta[1],
      submitted_.tran.z + delta[2]};
    if (config_.enable_y_limit) {
      const double bounded = config_.arm_is_right ? std::min(y_barrier_, planned_.tran.y) :
        std::max(y_barrier_, planned_.tran.y);
      if (bounded != planned_.tran.y) {
        planned_.tran.y = bounded;
        delta[1] = bounded - submitted_.tran.y;
        planned_velocity_[1] = 0; // hard workspace barrier takes precedence
      }
    }
    decision.increment.tran = {delta[0], delta[1], delta[2]};

    planned_rotation_ = submitted_rotation_;
    planned_angular_velocity_ = {{0, 0, 0}};
    if (config_.use_rotation) {
      Quaternion relative = quat_canonical(quat_normalized(quat_multiply(
        latest_input_.quat, quat_conjugate(controller_origin_.quat))));
      relative = quat_scaled_to_angle(relative, quat_rotation_angle(relative) * config_.rotation_scale);
      // Transform the desired rotation once, not the emitted error each cycle:
      // negating each RPY increment made the previous sign=-1 path diverge.
      if (config_.rotation_sign < 0) {relative = quat_conjugate(relative);}
      Quaternion candidate = quat_normalized(quat_multiply(relative, origin_rotation_));
      const double target_change = quat_rotation_angle(quat_normalized(quat_multiply(
        candidate, quat_conjugate(target_rotation_))));
      if (target_change >= config_.rotation_deadband_rad) {target_rotation_ = candidate;}
      const Vector3 remaining = rotation_vector(quat_multiply(
        target_rotation_, quat_conjugate(submitted_rotation_)));
      const double angle = vector_norm(remaining);
      const double speed_cap = std::min(config_.rotation_speed_rad_s,
        config_.rotation_max_step_rad / dt);
      double speed = speed_toward(angle, speed_cap, config_.rotation_accel_rad_s2, dt);
      // Same gain response across step_num values (gain is defined at 8 ms).
      const double gain = 1 - std::pow(1 - config_.rotation_gain, dt / 0.008);
      speed = std::min(speed, angle * gain / dt);
      Vector3 desired{{0, 0, 0}};
      if (angle > 1e-12) {
        for (size_t i = 0; i < 3; ++i) {desired[i] = remaining[i] * speed / angle;}
      }
      Vector3 change{{desired[0] - angular_velocity_[0], desired[1] - angular_velocity_[1],
        desired[2] - angular_velocity_[2]}};
      const double change_norm = vector_norm(change);
      const double factor = change_norm > 0 ? std::min(1.0,
        config_.rotation_accel_rad_s2 * dt / change_norm) : 0;
      Vector3 step{};
      for (size_t i = 0; i < 3; ++i) {
        planned_angular_velocity_[i] = angular_velocity_[i] + factor * change[i];
        step[i] = planned_angular_velocity_[i] * dt;
      }
      const Quaternion increment = quat_from_rotation_vector(step);
      decision.increment.rpy = rpy_from_quat(increment);
      // Accumulate the exact RPY representation passed to servo_p, not input.
      planned_rotation_ = quat_normalized(quat_multiply(
        quat_from_rpy(decision.increment.rpy), submitted_rotation_));
    }
    planned_.rpy = rpy_from_quat(planned_rotation_);
    target_.rpy = rpy_from_quat(target_rotation_);
  }

  Config config_;
  State state_ = State::Hold;
  std::string reason_ = "startup_hold";
  bool grip_ = false, pose_valid_ = false, feedback_valid_ = false, pending_ = false;
  double pose_time_ = -std::numeric_limits<double>::infinity();
  double grip_time_ = -std::numeric_limits<double>::infinity();
  double feedback_time_ = -std::numeric_limits<double>::infinity();
  double last_plan_time_ = std::numeric_limits<double>::quiet_NaN();
  ControllerPose latest_input_, controller_origin_;
  CartesianPose robot_origin_{}, submitted_{}, target_{}, planned_{}, feedback_{};
  Quaternion origin_rotation_ = quat_identity(), submitted_rotation_ = quat_identity();
  Quaternion target_rotation_ = quat_identity(), planned_rotation_ = quat_identity();
  Vector3 velocity_{{0, 0, 0}}, planned_velocity_{{0, 0, 0}};
  Vector3 angular_velocity_{{0, 0, 0}}, planned_angular_velocity_{{0, 0, 0}};
  Vector3 accepted_offset_{{0, 0, 0}};
  double y_barrier_ = 0;
  int fault_code_ = 0;
  double fault_time_ = 0;
  Decision pending_decision_;
};

}  // namespace jaka_teleop
#endif  // JAKA_DRIVER_TELEOP_CONTROL_HPP_
