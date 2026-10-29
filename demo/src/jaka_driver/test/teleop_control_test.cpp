#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "jaka_driver/teleop_control.hpp"

namespace
{
using namespace jaka_teleop;

void check(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}
bool near(double a, double b, double tolerance = 1e-7)
{return std::abs(a - b) <= tolerance;}
bool zero(const CartesianPose & pose)
{
  return near(pose.tran.x, 0) && near(pose.tran.y, 0) && near(pose.tran.z, 0) &&
    near(pose.rpy.rx, 0) && near(pose.rpy.ry, 0) && near(pose.rpy.rz, 0);
}
double rotation_error(const Rpy & a, const Rpy & b)
{
  return vector_norm(rotation_vector(quat_multiply(quat_from_rpy(a),
    quat_conjugate(quat_from_rpy(b)))));
}

// This fake is deliberately unrelated to JAKAZuRobot: no real SDK library is
// linked and the test cannot open a network connection or energize a device.
struct FakeSdk
{
  CartesianPose actual{};
  std::vector<CartesianPose> commands;
  int next_return = 0;
  double duration = 0;
  int collision_recover_calls = 0, clear_error_calls = 0;
  int power_on_calls = 0, enable_calls = 0;

  int servo_p(const CartesianPose & increment)
  {
    commands.push_back(increment);
    const int result = next_return;
    next_return = 0;
    if (result == 0) {
      actual.tran.x += increment.tran.x;
      actual.tran.y += increment.tran.y;
      actual.tran.z += increment.tran.z;
      actual.rpy = rpy_from_quat(quat_multiply(quat_from_rpy(increment.rpy),
        quat_from_rpy(actual.rpy)));
    }
    return result;
  }
};

struct Harness
{
  Config config;
  TeleopControl control;
  FakeSdk sdk;
  ControllerPose hand;
  double now = 100;
  bool grip = true;

  explicit Harness(const Config & c = Config()) : config(c), control(c) {}
  Decision tick(bool input = true, bool feedback = true)
  {
    if (input) {control.set_pose(hand, now); control.set_grip(grip, now);}
    if (feedback) {control.set_feedback(sdk.actual, now);}
    Decision d = control.plan(now);
    if (d.send) {control.commit(sdk.servo_p(d.increment), now + sdk.duration);}
    now += config.period_s;
    return d;
  }
  void repeat(int count)
  {for (int i = 0; i < count; ++i) {tick();}}
};

void continuous_hold_and_unique_decision()
{
  Harness h;
  h.tick();
  for (int i = 0; i < 200; ++i) {
    h.hand.tran.x = (i % 2) ? 0.2 : -0.2;
    Decision d = h.tick();
    check(d.send && zero(d.increment), "gripped stationary/deadband input sends a fresh zero");
    check(!h.control.plan(h.now - h.config.period_s).send, "same period cannot send twice");
  }
  h.grip = false;
  for (int i = 0; i < 100; ++i) {
    Decision d = h.tick();
    check(d.send && zero(d.increment), "released hold is continuous zero INCR");
  }
  check(h.sdk.commands.size() == 301, "one and only one SDK submission each simulated period");
  check(near(h.control.submitted().tran.x, 0), "stationary hold has zero cumulative increment");
  std::cout << "PASS continuous hold, deadband, unique sending\n";
}

void no_increment_replay_and_return_sequence()
{
  Harness h;
  h.tick();
  for (double position : {0., 20., 20., 15., 10., 5., 0.}) {
    h.hand.tran.x = position;
    h.tick();
  }
  h.repeat(1200);
  check(std::abs(h.control.submitted().tran.x) < 0.001, "[0,20,20,15,10,5,0] returns to origin");
  check(near(h.control.submitted().tran.x, h.sdk.actual.tran.x), "submitted state equals successful fake increments");
  const double settled = h.control.submitted().tran.x;
  h.repeat(200);
  check(near(settled, h.control.submitted().tran.x), "last nonzero increment cannot be replayed");
  check(zero(h.sdk.commands.back()), "repeated unchanged input eventually generates exact zero");
  std::cout << "PASS return sequence and no nonzero replay\n";
}

void limits_and_transaction()
{
  Config c;
  c.translation_deadband_mm = 0;
  Harness h(c);
  h.tick();
  h.hand.tran = {250, 125, -70};
  Vector3 prior{{0, 0, 0}};
  for (int i = 0; i < 1600; ++i) {
    Decision d = h.tick();
    const Vector3 velocity = h.control.velocity();
    for (size_t axis = 0; axis < 3; ++axis) {
      check(std::abs(velocity[axis]) <= c.translation_speed_mm_s + 1e-7, "translation speed cap");
      check(std::abs(velocity[axis] - prior[axis]) <= c.translation_accel_mm_s2*c.period_s + 1e-7,
        "translation acceleration cap");
    }
    prior = velocity;
    check(near(h.control.planned().tran.x, h.control.submitted().tran.x), "successful planned/submitted state consistent");
    check(near(h.control.target().tran.x, 250), "rate limit does not discard target residual");
    check(near(h.control.submitted().tran.x, h.sdk.actual.tran.x), "command reference advances by actual submitted increment");
    check(std::abs(d.increment.tran.x) <= c.translation_speed_mm_s*c.period_s + 1e-7, "step bound uses servo horizon");
  }
  check(near(h.control.submitted().tran.x, 250, 0.001), "trajectory reaches capped target after enough cycles");

  h.hand.tran.x += 20;
  h.control.set_pose(h.hand, h.now);
  h.control.set_grip(true, h.now);
  h.control.set_feedback(h.sdk.actual, h.now);
  const double before = h.control.submitted().tran.x;
  Decision d = h.control.plan(h.now);
  check(d.send, "transaction is planned");
  check(near(h.control.submitted().tran.x, before), "planning alone does not advance submitted state");
  check(!h.control.plan(h.now).send, "uncommitted transaction cannot submit twice");
  h.control.commit(-12, h.now);
  check(h.control.state() == State::Fault, "SDK failure latches fault");
  check(near(h.control.submitted().tran.x, before), "failed submission leaves reference unchanged");
  check(near(h.control.planned().tran.x, before), "failed plan is discarded");
  check(near(h.control.target().tran.x, before), "failed target residual is cleared");
  std::cout << "PASS limits, target residual and transactional commit\n";
}

void release_and_reengage()
{
  Harness h;
  h.sdk.actual.tran = {10, 20, 30};
  h.tick();
  h.hand.tran.x = 500;
  h.hand.quat = quat_from_rpy({0.5, 0.4, -0.7});
  h.repeat(12);
  check(h.control.target().tran.x - h.control.submitted().tran.x > 450, "release test has large residual");
  check(rotation_error(h.control.target().rpy, h.control.submitted().rpy) > 0.1, "release test has rotation residual");
  h.grip = false;
  Decision released = h.tick();
  check(zero(released.increment), "release immediately clears translation and rotation residual");
  const CartesianPose held = h.control.submitted();
  h.hand.tran.x = -1000;
  h.hand.quat = quat_from_rpy({-0.8, 0.3, 0.8});
  h.repeat(50);
  check(near(h.control.submitted().tran.x, held.tran.x), "old target never pursued after release");
  h.sdk.actual.tran.x += 2; // feedback can differ from the submitted command
  h.grip = true;
  check(zero(h.tick().increment), "reengagement starts at fresh hand/actual feedback reference");
  check(near(h.control.submitted().tran.x, held.tran.x + 2), "feedback is used only to establish new reference");
  h.hand.tran.x += 5;
  h.repeat(150);
  check(near(h.control.submitted().tran.x, held.tran.x + 7, 0.001), "new clutch follows only new displacement");
  std::cout << "PASS residual release and feedback-based reengagement\n";
}

void stale_input_tracking_and_delays()
{
  Harness h;
  h.tick();
  h.hand.tran.x = 100;
  h.repeat(5);
  h.now += 0.25;
  Decision expired = h.tick(false);
  check(expired.send && zero(expired.increment), "gap skips old input and sends zero without catchup");
  check(h.control.state() == State::AwaitRelease, "timeout requires explicit release before reengagement");
  h.repeat(20);
  check(h.control.state() == State::AwaitRelease, "continued held Grip cannot reengage after timeout");
  h.grip = false; h.tick();
  h.grip = true; check(zero(h.tick().increment), "fresh release/repress reanchors after timeout");
  h.control.set_pose(h.hand, h.now, false);
  check(h.control.state() == State::AwaitRelease, "explicit tracking loss clears control immediately");

  Harness delayed;
  delayed.tick();
  delayed.hand.tran.x = 10;
  delayed.sdk.duration = 0.1;
  delayed.tick();
  check(delayed.control.state() == State::AwaitRelease, "long successful SDK call inhibits further tracking");
  check(delayed.control.submitted().tran.x > 0, "successful slow call remains a submitted increment");
  delayed.sdk.duration = 0;
  check(zero(delayed.tick().increment), "after slow call no delayed target pursuit");

  Harness feedback;
  feedback.tick();
  for (int i = 0; i < 60; ++i) {feedback.tick(true, false);}
  check(feedback.control.state() == State::AwaitRelease, "stale robot feedback inhibits tracking");
  std::cout << "PASS stale input, tracking loss, servo delay and feedback age\n";
}

void persistent_fault_and_external_pause()
{
  Harness h;
  h.tick();
  h.sdk.next_return = -3;
  h.hand.tran.x = 10;
  h.tick();
  const size_t sent = h.sdk.commands.size();
  const double fault_time = h.control.fault_time();
  check(h.control.fault_code() == -3 && h.control.reason() == "servo_p", "raw original SDK fault preserved");
  for (int i = 0; i < 60; ++i) {
    h.grip = (i % 2) != 0;
    h.tick();
  }
  h.control.fault(0x128000, h.now, "later_fault");
  check(h.control.fault_code() == -3 && h.control.fault_time() == fault_time, "first fault remains latched");
  check(h.sdk.commands.size() == sent, "fault stops accepting all servo decisions");
  check(h.sdk.collision_recover_calls == 0 && h.sdk.clear_error_calls == 0 &&
    h.sdk.power_on_calls == 0 && h.sdk.enable_calls == 0, "Grip never invokes recovery/power/enable");

  Harness paused;
  paused.tick();
  paused.hand.tran.x = 50;
  paused.repeat(3);
  paused.control.external_pause("gripper_serial_operation");
  check(zero(paused.tick().increment), "blocking serial operation clears trajectory residual");
  paused.control.fault(-1001, paused.now + 0.3, "gripper_call_deadline");
  paused.repeat(30);
  check(paused.control.state() == State::Fault, "excessive gripper delay can latch owner fault");

  Harness repeated_release;
  repeated_release.grip = false;
  repeated_release.tick();
  repeated_release.control.external_pause("maintenance_requires_new_release");
  repeated_release.control.set_grip(false, repeated_release.now-repeated_release.config.period_s);
  check(repeated_release.control.state() == State::AwaitRelease,
    "replaying a cached release does not acknowledge a later interruption");
  std::cout << "PASS persistent fault, no Grip recovery and external serial pause\n";
}

void rotation_regression()
{
  for (int sign : {1, -1}) {
    Config c;
    c.rotation_sign = sign;
    Harness a(c), b(c);
    a.sdk.actual.rpy = b.sdk.actual.rpy = {0.8, -0.4, 1.1};
    a.hand.quat = b.hand.quat = quat_from_rpy({-0.3, 0.2, 0.6});
    a.tick(); b.tick();
    const Quaternion hand_origin = a.hand.quat;
    const Quaternion relative = quat_from_rpy({0.2, -0.15, 0.1});
    a.hand.quat = quat_multiply(relative, hand_origin);
    b.hand.quat = {-a.hand.quat.s, -a.hand.quat.x, -a.hand.quat.y, -a.hand.quat.z};
    // Rotation and translation noise are independent.
    a.hand.tran.x = b.hand.tran.x = 0.2;
    Vector3 previous{{0, 0, 0}};
    for (int i = 0; i < 2500; ++i) {
      Decision da = a.tick(), db = b.tick();
      check(near(da.increment.tran.x, 0), "rotation must not admit sub-deadband translation noise");
      check(rotation_error(da.increment.rpy, db.increment.rpy) < 1e-7, "q and -q emit equivalent orientation increments");
      const Vector3 velocity = a.control.angular_velocity();
      Vector3 change{{velocity[0]-previous[0], velocity[1]-previous[1], velocity[2]-previous[2]}};
      check(vector_norm(velocity) <= c.rotation_speed_rad_s + 1e-8, "angular speed norm bounded");
      check(vector_norm(change) <= c.rotation_accel_rad_s2*c.period_s + 1e-8, "angular acceleration norm bounded");
      check(vector_norm(rotation_vector(quat_from_rpy(da.increment.rpy))) <= c.rotation_max_step_rad + 1e-8,
        "legacy angular increment cap retained");
      previous = velocity;
    }
    check(rotation_error(a.control.target().rpy, a.control.submitted().rpy) < 1e-6,
      "signed nonzero-initial-pose rotation converges without sign=-1 divergence");
    check(rotation_error(a.control.submitted().rpy, a.sdk.actual.rpy) < 1e-6,
      "submitted quaternion composition agrees with independently accumulated fake motion");

    // Independent physical direction expectation: an initial yaw of +0.6,
    // followed by a +0.4 hand yaw and scale 0.5, ends at 0.6 +/- 0.2.
    Harness yaw(c);
    yaw.sdk.actual.rpy.rz = 0.6;
    yaw.hand.quat = quat_from_rpy({0, 0, -0.3});
    yaw.tick();
    yaw.hand.quat = quat_from_rpy({0, 0, 0.1});
    yaw.repeat(1800);
    check(near(yaw.control.submitted().rpy.rz, 0.6 + sign*0.2, 1e-6),
      "positive hand yaw maps to the explicitly expected signed robot yaw");
  }
  // Input crosses Euler +/-pi while representing a small physical change.
  Harness edge;
  edge.hand.quat = quat_from_rpy({0, 0, 3.13});
  edge.tick();
  edge.hand.quat = quat_from_rpy({0, 0, -3.13});
  edge.repeat(500);
  check(rotation_error(edge.control.target().rpy, edge.control.submitted().rpy) < 1e-6,
    "Euler wrap boundary remains a short relative rotation");

  // Reproduce an independent limitation without silently redefining the
  // existing controller-relative, shortest-arc scaling semantics. Crossing pi
  // relative to the clutch origin is different from the small Euler wrap above.
  constexpr double pi = 3.14159265358979323846;
  Harness half_turn;
  half_turn.tick(); // identity hand orientation is the clutch origin
  half_turn.hand.quat = quat_from_rpy({0, 0, 179*pi/180});
  half_turn.tick();
  const double before_crossing = half_turn.control.target().rpy.rz;
  half_turn.hand.quat = quat_from_rpy({0, 0, 181*pi/180});
  const Decision after = half_turn.tick();
  const double after_crossing = half_turn.control.target().rpy.rz;
  check(near(before_crossing, 89.5*pi/180, 1e-8) &&
    near(after_crossing, -89.5*pi/180, 1e-8),
    "known shortest-arc scaling limitation: +179/+181 maps to +89.5/-89.5 degrees");
  check(vector_norm(rotation_vector(quat_from_rpy(after.increment.rpy))) <=
    half_turn.config.rotation_max_step_rad + 1e-8,
    "known target discontinuity does not bypass emitted angular step bound");
  std::cout << "KNOWN LIMITATION: relative hand yaw +179/+181 deg maps to target +89.5/-89.5 deg at scale=0.5; emitted step remains bounded.\n";
  std::cout << "PASS quaternion signs, nonzero origins, rotation sign, independent noise and angular limits\n";
}

void y_boundary_and_failure()
{
  for (bool right : {false, true}) {
    Config c;
    c.arm_is_right = right;
    Harness h(c);
    const double direction = right ? 1 : -1;
    h.sdk.actual.tran.y = direction * 100;
    h.tick();
    h.hand.tran.y = direction * 1000;
    for (int i = 0; i < 700; ++i) {
      h.tick();
      check(direction * h.control.target().tran.y <= 110 + 1e-7, "Y target in shared command frame clamped");
      check(direction * h.control.planned().tran.y <= 110 + 1e-7, "Y planned target bounded");
      check(direction * h.control.submitted().tran.y <= 110 + 1e-7, "Y successful commands bounded");
    }
    check(near(direction*h.control.submitted().tran.y, 110, 0.001), "Y reaches boundary without drift");
    h.grip = false; h.tick();
    h.hand.tran.y = -2000;
    h.grip = true;
    check(zero(h.tick().increment), "Y reengagement at boundary resets hand reference");
    h.hand.tran.y -= direction * 20;
    h.sdk.next_return = -12;
    const double before = h.control.submitted().tran.y;
    h.tick();
    check(near(h.control.submitted().tran.y, before) && near(h.control.target().tran.y, before),
      "Y failure cannot advance command reference or retain stale target");
  }
  Harness outside;
  outside.sdk.actual.tran.y = -120;
  outside.tick();
  check(zero(outside.tick().increment), "engagement beyond Y bound does not synthesize pullback");
  outside.hand.tran.y = -20;
  outside.repeat(200);
  check(near(outside.control.submitted().tran.y, -120), "outside boundary forbids further outward motion");
  outside.hand.tran.y = 20;
  outside.repeat(300);
  check(outside.control.submitted().tran.y > -120, "outside boundary permits return inward");
  std::cout << "PASS Y target/planned/commit boundary, reengagement and failed command\n";
}

void malformed_input_and_period()
{
  Harness invalid;
  invalid.tick();
  invalid.hand.tran.x = std::numeric_limits<double>::quiet_NaN();
  check(zero(invalid.tick().increment), "invalid translation produces hold");
  check(invalid.control.state() == State::AwaitRelease, "invalid translation inhibits tracking");
  Config c;
  c.period_s = 0.016;
  c.max_interval_s = 0.064;
  Harness h(c);
  h.tick();
  h.hand.tran.x = 20;
  const Decision d = h.tick();
  check(near(d.increment.tran.x, c.translation_accel_mm_s2*c.period_s*c.period_s, 1e-7),
    "step_num=2 uses its actual 16ms horizon for acceleration and distance");
  bool rejected = false;
  try {c.period_s = -1; TeleopControl bad(c);}
  catch (const std::invalid_argument &) {rejected = true;}
  check(rejected, "invalid motion/timing configuration rejected");
  Harness startup;
  startup.control.external_pause("startup_requires_release");
  startup.control.set_grip(false, -1);
  startup.control.set_grip(false, std::numeric_limits<double>::quiet_NaN());
  check(startup.control.state() == State::AwaitRelease, "uninitialized mailbox cannot bypass startup release gate");
  startup.repeat(10);
  check(startup.control.state() == State::AwaitRelease, "optional startup gate rejects already-held Grip");
  startup.grip = false; startup.tick();
  startup.grip = true; startup.tick();
  check(startup.control.state() == State::Tracking, "startup release/fresh press engages");
  std::cout << "PASS invalid input, configuration and 16ms timebase\n";
}
}  // namespace

int main()
{
  continuous_hold_and_unique_decision();
  no_increment_replay_and_return_sequence();
  limits_and_transaction();
  release_and_reengage();
  stale_input_tracking_and_delays();
  persistent_fault_and_external_pause();
  rotation_regression();
  y_boundary_and_failure();
  malformed_input_and_period();
  std::cout << "All 9 offline teleop control scenario groups passed.\n";
}
