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
  c.translation_filter_tau_s = 0;
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
  check(near(h.control.submitted().tran.x, held.tran.x), "later differing feedback cannot erase the accepted command epoch");
  h.hand.tran.x += 5;
  h.repeat(150);
  check(near(h.control.submitted().tran.x, held.tran.x + 4.5, 0.001), "new clutch follows new displacement with fixed soft-deadzone bias");
  std::cout << "PASS residual release and continuous command-epoch reengagement\n";
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

  // A half-turn sample is outside the enforced relative hand workspace.
  // Both signs must hold rather than reinterpreting it on the opposite arc.
  Harness half_turn;
  half_turn.tick();
  half_turn.hand.quat = quat_from_rpy({0,0,3.12413936106985});
  const Decision refused = half_turn.tick();
  check(half_turn.control.state()==State::AwaitRelease && zero(refused.increment),
    "relative near-pi orientation is refused before shortest-arc scaling");
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
  c.translation_filter_tau_s = 0;
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

void cached_feedback_reengagement()
{
  Config c;
  c.arm_is_right = true;
  Harness h(c);
  h.sdk.actual.tran.y = 100;
  h.tick(); // one feedback sample, next arrives about 100 ms later
  h.hand.tran.y = 20;
  for (int i=0;i<10;++i) {h.tick(true,false);}
  const double before=h.control.submitted().tran.y;
  check(near(before,101.76,1e-6), "repro reaches 101.76 mm before second feedback sample");
  h.grip=false; h.tick(true,false);
  h.grip=true; h.tick(true,false);
  std::cout << "REENGAGE submitted before=" << before << " after=" << h.control.submitted().tran.y
    << " executed=" << h.sdk.actual.tran.y << '\n';
  check(near(h.control.submitted().tran.y,before), "reengagement cannot reset accepted INCR history to cached feedback");
  h.hand.tran.y += 20;
  for (int i=0;i<300;++i) {
    // A 13-cycle SDK query interval is 104 ms; its cache is additionally
    // delayed by 80 ms of execution. Queries are not synchronization barriers.
    if (i%13==0) {
      CartesianPose stale=h.sdk.actual; stale.tran.y=std::max(100.,stale.tran.y-2.);
      h.control.set_feedback(stale,h.now);
    }
    h.tick(true,false);
    check(h.sdk.actual.tran.y<=110.+1e-7, "physical sum of accepted INCRs never crosses Y=110 under delayed feedback");
    check(near(h.sdk.actual.tran.y,h.control.submitted().tran.y), "command epoch and cumulative accepted increments remain aligned");
  }

  TeleopControl delayed(c);
  ControllerPose hand;
  CartesianPose executed{}; executed.tran.y=100;
  double now=200,accepted_y=100;
  bool held=true;
  std::vector<std::pair<int,CartesianPose>> queue;
  for(int cycle=0;cycle<450;++cycle) {
    // Execution occurs 80ms after successful SDK acceptance. Robot feedback
    // updates every104ms. Release/repress twice between feedback updates.
    for(const auto &pending:queue) if(pending.first==cycle) executed.tran.y+=pending.second.tran.y;
    if(cycle%13==0) delayed.set_feedback(executed,now);
    if(cycle==1 || cycle==12 || cycle==25) hand.tran.y+=20;
    held=cycle!=8 && cycle!=21;
    delayed.set_pose(hand,now);delayed.set_grip(held,now);
    const Decision d=delayed.plan(now);
    check(d.send,"delayed model still generates exactly one command per period");
    delayed.commit(0,now);
    accepted_y+=d.increment.tran.y;
    queue.push_back({cycle+10,d.increment});
    check(near(accepted_y,delayed.submitted().tran.y),"lagging execution cannot overwrite accepted command history");
    check(accepted_y<=110.+1e-7 && executed.tran.y<=110.+1e-7,"Y bound applies to both accepted and delayed-executed trajectories");
    now+=c.period_s;
  }
  check(near(executed.tran.y,110,.001),"delayed queued execution settles at the real Y boundary");
  std::cout << "PASS cached 100ms feedback and 80ms delayed execution cannot roll back clutch reference\n";
}

void continuous_slow_translation()
{
  Harness h;
  h.tick();
  double minimum=1e9, maximum=-1e9, sum=0, square=0;
  int count=0;
  for(int i=1;i<=750;++i) {
    h.hand.tran.x=5.*i*h.config.period_s;
    const Decision d=h.tick();
    const double v=d.increment.tran.x/h.config.period_s;
    if(i>=125) {minimum=std::min(minimum,v);maximum=std::max(maximum,v);sum+=v;square+=v*v;++count;}
  }
  const double mean=sum/count;
  const double rms=std::sqrt(std::max(0.,square/count-mean*mean));
  std::cout << "SLOW 5mm/s after1s min=" << minimum << " max=" << maximum
    << " mean=" << mean << " ripple_rms=" << rms << " mm/s\n";
  check(minimum>4.9 && maximum<5.1 && rms<.05, "5mm/s tracking must not repeatedly stall and jump at deadband threshold");
  h.repeat(300);
  check(zero(h.sdk.commands.back()), "constant accepted hand position settles to new-zero hold");
  const double centre=h.control.submitted().tran.x;
  double peak_noise=0,first_mean=0,last_mean=0;
  for(int i=0;i<5000;++i) {
    h.hand.tran.x=30.+(i%2 ? .2 : -.2);h.tick();
    peak_noise=std::max(peak_noise,std::abs(h.control.submitted().tran.x-centre));
    if(i>=1000 && i<2000) first_mean+=h.control.submitted().tran.x/1000.;
    if(i>=4000) last_mean+=h.control.submitted().tran.x/1000.;
  }
  std::cout << "STATIC away-origin +/-0.2mm noise peak=" << peak_noise
    << " block_mean_difference=" << last_mean-first_mean << " mm\n";
  check(peak_noise<.1,
    "away-origin static noise remains absolutely bounded and cannot accumulate a random walk");
  h.hand.tran.x=30.;h.repeat(500);
  check(std::abs(h.control.submitted().tran.x-centre)<.001,
    "stationary away-origin noise leaves no permanent command offset when noise returns to centre");
  std::cout << "STATIC returned-to-centre error=" << h.control.submitted().tran.x-centre << " mm\n";
  for(int i=749;i>=0;--i) {h.hand.tran.x=5.*i*h.config.period_s;h.tick();}
  h.repeat(500);
  check(std::abs(h.control.submitted().tran.x)<.001, "slow out-and-back has less than 0.001mm final command error");
  h.hand.tran.x=500;h.repeat(4);h.grip=false;
  check(zero(h.tick().increment), "slow-motion filter cannot retain a release residual");
  const double held=h.control.submitted().tran.x;h.repeat(500);
  check(near(held,h.control.submitted().tran.x), "release clears the slow-motion trajectory residual");
  Harness noise;
  noise.tick();
  for(int i=0;i<5000;++i) {noise.hand.tran.x=(i%2 ? .2 : -.2);noise.tick();}
  check(near(noise.control.submitted().tran.x,0), "bounded stationary input noise has no cumulative drift");
  std::cout << "PASS continuous slow speed, static noise, roundtrip and residual release\n";
}

void rotation_workspace_reengagement()
{
  constexpr double pi=3.14159265358979323846;
  for(int sign:{1,-1}) {
    Harness h;
    h.sdk.actual.rpy={.5,-.6,.9};
    h.hand.quat=quat_from_rpy({.4,.2,-.7});
    h.tick();
    const Quaternion origin=h.hand.quat;
    for(double degrees:{120.,160.,169.,179.,181.,179.,160.}) {
      h.hand.quat=quat_multiply(quat_from_rpy({0,0,sign*degrees*pi/180}),origin);
      h.hand.quat={-h.hand.quat.s,-h.hand.quat.x,-h.hand.quat.y,-h.hand.quat.z};
      const Decision d=h.tick();
      if(degrees>=179 || h.control.state()==State::AwaitRelease) {
        check(h.control.state()==State::AwaitRelease && zero(d.increment),
          "crossing relative attitude workspace must hold instead of pursuing opposite shortest-arc target");
      }
    }
    check(h.control.state()==State::AwaitRelease,"return from boundary while held cannot silently resume");
    h.grip=false;h.tick();h.grip=true;
    check(zero(h.tick().increment),"workspace re-clutch uses current hand orientation without old residual");
    h.hand.quat=quat_multiply(quat_from_rpy({0,.04,0}),h.hand.quat);
    check(h.tick().send && h.control.state()==State::Tracking,"new small rotation works after release/repress");
  }

  Harness dropped;
  dropped.tick();
  dropped.hand.quat=quat_from_rpy({0,0,169*pi/180});dropped.tick();
  dropped.hand.quat=quat_from_rpy({0,0,191*pi/180});
  check(zero(dropped.tick().increment) && dropped.control.state()==State::AwaitRelease,
    "dropped-frame path across pi cannot bypass the workspace with two inside endpoints");

  TeleopControl queued;
  ControllerPose frame;
  queued.set_feedback({},300.);
  check(queued.set_ordered_input(frame,true,300.,300.,300.2,1),"queued workspace fixture starts with a complete frame");
  check(queued.plan(300.).send,"queued workspace fixture sends initial hold");queued.commit(0,300.);
  frame.quat=quat_from_rpy({0,0,179*pi/180});
  queued.set_ordered_input(frame,true,300.004,300.004,300.2,2);
  frame.quat=quat_from_rpy({0,0,160*pi/180});
  queued.set_ordered_input(frame,true,300.006,300.006,300.2,3);
  const Decision after_multiple=queued.plan(300.008);
  check(after_multiple.send && zero(after_multiple.increment) && queued.state()==State::AwaitRelease,
    "an intermediate complete frame outside workspace cannot be overwritten by an inside frame before one plan");
  std::cout << "PASS enforced relative rotation workspace, q/-q, nonzero origins, path crossing and re-clutch\n";
}


void source_deadline_at_execution()
{
  Harness h;
  h.now=100.144;
  h.tick();
  // Source frame produced at100.0 reaches host at100.150: receive-age budget
  // remains200ms, but its source budget expires at100.200, only50ms later.
  h.hand.tran.x=20.;
  h.control.set_pose(h.hand,100.150);
  h.control.set_grip(true,100.150);
  h.control.set_input_deadline(100.200);
  for(double t:{100.152,100.160,100.168,100.176,100.184,100.192,100.200}) {
    h.now=t;const Decision d=h.tick(false);
    check(d.send && h.control.state()==State::Tracking,
      "source remains executable through its exact monotonic deadline");
  }
  h.now=100.208;
  const Decision expired=h.tick(false);
  check(expired.send && zero(expired.increment) && h.control.state()==State::AwaitRelease,
    "source deadline stops motion even though receipt age remains below input_timeout");
  h.grip=false;h.tick();h.grip=true;
  check(zero(h.tick().increment) && h.control.state()==State::AwaitRelease,
    "expired source deadline also prevents clutch reengagement");
  h.control.set_input_deadline(h.now+.1);
  h.grip=false;h.tick();h.grip=true;h.tick();
  check(h.control.state()==State::Tracking,"new source deadline plus actual release permits new reference");
  std::cout << "PASS execution-time source deadline distinct from receive TTL\n";
}


void source_ordered_input_preserves_receive_age()
{
  Config config;
  config.max_interval_s=1.;
  TeleopControl c(config);
  ControllerPose first,second;
  second.tran.x=20.;
  c.external_pause("requires_initial_release");
  check(c.set_ordered_input(first,false,200.010,200.011,200.050,1),"first source-ordered release frame admitted");
  check(c.set_ordered_input(second,true,200.005,200.006,200.050,2),"newer source frame can carry earlier original receipt times");
  check(near(c.latest_input().tran.x,20.),"latest complete source frame replaces the pose despite reversed callback receipt order");
  c.set_feedback({},200.020);
  Decision d=c.plan(200.020);check(d.send && zero(d.increment),"ordered reengagement creates a zero-displacement origin");c.commit(0,200.020);
  second.tran.x=30.;
  check(c.set_ordered_input(second,true,200.014,200.015,200.050,3),"subsequent ordered motion frame admitted");
  d=c.plan(200.028);check(d.send && d.increment.tran.x>0,"newer ordered pose drives motion");c.commit(0,200.028);
  c.external_pause("requires_new_release");
  check(!c.set_ordered_input(first,false,200.030,200.031,200.200,3),"replayed source sequence ignored even with fabricated newer receipt times");
  check(c.state()==State::AwaitRelease,"replaying a release cannot acknowledge a new interruption");
  check(c.set_ordered_input(second,false,200.016,200.017,200.050,4),"new source release acknowledges the pause");
  check(c.set_ordered_input(second,true,200.012,200.013,200.050,5),"later source press retains its earlier raw receipt age");
  d=c.plan(200.036);check(d.send && zero(d.increment),"reengagement rebases to newest ordered hand frame");c.commit(0,200.036);
  second.tran.x=50.;
  check(c.set_ordered_input(second,true,200.012,200.013,200.050,6),"source ordering does not grant a new receive lifetime");
  d=c.plan(200.044);check(d.send && d.increment.tran.x>0,"source before original deadline remains executable");c.commit(0,200.044);
  d=c.plan(200.052);check(d.send && zero(d.increment) && c.state()==State::AwaitRelease,"original source deadline expires regardless of ordered assembly time");c.commit(0,200.052);
  first.tran.x=std::numeric_limits<double>::quiet_NaN();
  check(!c.set_ordered_input(first,false,200.060,200.061,200.150,7) && c.state()==State::AwaitRelease,"malformed ordered release cannot acknowledge interruption");
  check(!c.set_ordered_input(second,false,200.062,200.063,200.150,6),"older sequence cannot replace an invalid newer frame");

  TeleopControl receive_ttl(config);
  receive_ttl.set_feedback({},200.020);
  check(receive_ttl.set_ordered_input(second,true,200.005,200.006,200.500,1),"long source lifetime does not replace raw receive TTL");
  d=receive_ttl.plan(200.020);receive_ttl.commit(0,200.020);
  second.tran.x+=20.;
  check(receive_ttl.set_ordered_input(second,true,200.007,200.008,200.500,2),"new ordered sample preserves early raw receive times");
  d=receive_ttl.plan(200.028);receive_ttl.commit(0,200.028);
  receive_ttl.set_feedback({},200.208);
  d=receive_ttl.plan(200.208);
  check(d.send && zero(d.increment) && receive_ttl.state()==State::AwaitRelease,"receive TTL expires from callback arrival, not assembly/processing time");
  std::cout << "PASS source-ordered input with reversed receipt order, replay rejection and original age budgets\n";
}

}  // namespace

int main(int argc, char **argv)
{
  if(argc>1) {
    const std::string name=argv[1];
    if(name=="cached_feedback") cached_feedback_reengagement();
    else if(name=="slow_translation") continuous_slow_translation();
    else if(name=="rotation_workspace") rotation_workspace_reengagement();
    else if(name=="ordered_input") source_ordered_input_preserves_receive_age();
    else return 2;
    return 0;
  }
  continuous_hold_and_unique_decision();
  no_increment_replay_and_return_sequence();
  limits_and_transaction();
  release_and_reengage();
  stale_input_tracking_and_delays();
  persistent_fault_and_external_pause();
  rotation_regression();
  y_boundary_and_failure();
  malformed_input_and_period();
  cached_feedback_reengagement();
  continuous_slow_translation();
  rotation_workspace_reengagement();
  source_deadline_at_execution();
  source_ordered_input_preserves_receive_age();
  std::cout << "All 14 offline teleop control scenario groups passed.\n";
}
