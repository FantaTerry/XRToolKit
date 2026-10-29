#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/pgi140_gripper.hpp"
#include "jaka_driver/teleop_control.hpp"

namespace {
using Clock = std::chrono::steady_clock;
double steady_seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
template<class Stamp> double stamp_seconds(const Stamp &s) { return s.sec + s.nanosec * 1e-9; }
constexpr int kTimingFault = -10001, kFeedbackFault = -10002, kGripperFault = -10003;

// Each arm has one process and one SDK owner. Callbacks write the mailbox only.
// Grip never invokes recovery, power or enable; controller recovery + process
// restart is a separate, explicit operator workflow.
class RcControlNode : public rclcpp::Node {
public:
  RcControlNode() : Node("rc_ctrl_node") {
    arm_ = declare_parameter<std::string>("arm_name", "left");
    if (arm_ != "left" && arm_ != "right") throw std::invalid_argument("arm_name");
    const bool right = arm_ == "right";
    ip_ = declare_parameter<std::string>("robot_ip", right ? "192.168.71.36" : "192.168.71.37");
    auto target = declare_parameter<std::string>("target_topic", "/rc_ctrl/" + arm_ + "_target");
    auto buttons = declare_parameter<std::string>("button_topic", "/rc_ctrl/button");
    auto tracking = declare_parameter<std::string>("tracking_topic", "/rc_ctrl/" + arm_ + "_tracking");
    grip_axis_ = declare_parameter<int>("grip_axis", right ? 1 : 0);
    trigger_axis_ = declare_parameter<int>("gripper_axis", right ? 3 : 2);
    int button = declare_parameter<int>("gripper_button_index", -1);
    trigger_button_ = button < 0 ? (right ? 11 : 10) : button;
    linear_ = declare_parameter<bool>("gripper_linear", true);
    if (declare_parameter<bool>("gripper_use_button", false)) linear_ = false;
    trigger_deadband_ = declare_parameter<double>("gripper_trigger_deadband", .02);
    press_ = declare_parameter<double>("grip_press_threshold", .6);
    release_ = declare_parameter<double>("grip_release_threshold", .4);
    if (!(release_ >= 0 && press_ <= 1 && press_ > release_) || grip_axis_ < 0 ||
        trigger_axis_ < 0 || trigger_button_ < 0 || !std::isfinite(trigger_deadband_) || trigger_deadband_ <= 0)
      throw std::invalid_argument("invalid Grip/Trigger parameters");
    (void)declare_parameter<bool>("debug", false);
    step_num_ = declare_parameter<int>("servo_step_num", 1);
    if (step_num_ < 1 || step_num_ > 10) throw std::invalid_argument("servo_step_num 1..10");
    cfg_.period_s = step_num_ * .008;
    cfg_.max_interval_s = declare_parameter<double>("max_servo_interval_s", .032);
    cfg_.input_timeout_s = declare_parameter<double>("input_timeout_s", .2);
    cfg_.feedback_timeout_s = declare_parameter<double>("feedback_timeout_s", .3);
    cfg_.translation_speed_mm_s = declare_parameter<double>("translation_speed_mm_s", 125.);
    cfg_.translation_accel_mm_s2 = declare_parameter<double>("translation_accel_mm_s2", 500.);
    cfg_.translation_deadband_mm = declare_parameter<double>("translation_deadband_mm", .5);
    cfg_.use_rotation = declare_parameter<bool>("use_rpy_ctrl", true);
    cfg_.rotation_speed_rad_s = declare_parameter<double>("rotation_speed_rad_s", .1875);
    cfg_.rotation_accel_rad_s2 = declare_parameter<double>("rotation_accel_rad_s2", .75);
    cfg_.rotation_scale = declare_parameter<double>("rotation_scale", .5);
    cfg_.rotation_gain = declare_parameter<double>("rotation_gain", .5);
    cfg_.rotation_deadband_rad = declare_parameter<double>("rotation_deadband_rad", .005);
    cfg_.rotation_max_step_rad = declare_parameter<double>("rotation_max_step_rad", .0015);
    cfg_.rotation_sign = declare_parameter<int>("rotation_sign", 1);
    cfg_.enable_y_limit = declare_parameter<bool>("enable_y_limit", true);
    cfg_.arm_is_right = right;
    cfg_.y_limit_mm = declare_parameter<double>("y_limit_mm", 110.);
    feedback_period_ = declare_parameter<double>("feedback_period_s", .1);
    gripper_timeout_ = declare_parameter<double>("gripper_call_timeout_s", .25);
    require_tracking_ = declare_parameter<bool>("require_tracking", true);
    source_age_ = declare_parameter<double>("max_source_age_s", .2);
    if (!std::isfinite(feedback_period_) || feedback_period_ <= 0 || feedback_period_ >= cfg_.feedback_timeout_s ||
        !std::isfinite(gripper_timeout_) || gripper_timeout_ <= 0 || !std::isfinite(source_age_) || source_age_ <= 0)
      throw std::invalid_argument("invalid freshness/SDK timing parameters");
    control_ = std::make_unique<jaka_teleop::TeleopControl>(cfg_);
    control_->external_pause("startup_requires_release");
    filter_ = declare_parameter<std::string>("servo_filter", "carte_nlf");
    nlf_v_ = declare_parameter<double>("nlf_translation_speed_mm_s", 1500.);
    nlf_a_ = declare_parameter<double>("nlf_translation_accel_mm_s2", 2500.);
    nlf_j_ = declare_parameter<double>("nlf_translation_jerk_mm_s3", 2800.);
    nlf_rv_ = declare_parameter<double>("nlf_rotation_speed_deg_s", 1120.);
    nlf_ra_ = declare_parameter<double>("nlf_rotation_accel_deg_s2", 1180.);
    nlf_rj_ = declare_parameter<double>("nlf_rotation_jerk_deg_s3", 1720.);
    foresight_buf_ = declare_parameter<int>("foresight_buffer", 10);
    foresight_kp_ = declare_parameter<double>("foresight_kp", .5);
    if (filter_ != "carte_nlf" && filter_ != "none" && filter_ != "speed_foresight")
      throw std::invalid_argument("servo_filter: carte_nlf, none or speed_foresight");
    for (double v : {nlf_v_, nlf_a_, nlf_j_, nlf_rv_, nlf_ra_, nlf_rj_, foresight_kp_})
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid filter parameter");
    if (foresight_buf_ < 3 || foresight_buf_ > 100) throw std::invalid_argument("foresight_buffer 3..100");
    pgi140::GripperConfig gc;
    gc.rs485_channel = declare_parameter<int>("gripper_rs485_channel", 1);
    gc.slave_id = declare_parameter<int>("gripper_slave_id", 1);
    gc.baudrate = declare_parameter<int>("gripper_baudrate", 115200);
    gc.force_percent = declare_parameter<int>("gripper_force", 20);
    gc.speed_percent = declare_parameter<int>("gripper_speed", 100);
    gc.open_position = declare_parameter<int>("gripper_open_position", 1000);
    gc.closed_position = declare_parameter<int>("gripper_closed_position", 0);
    gc.initialize = declare_parameter<bool>("gripper_initialize", true);
    gc.initialize_command = declare_parameter<int>("gripper_initialize_command", 1);
    gc.initialize_delay_ms = declare_parameter<int>("gripper_initialize_delay_ms", 2000);
    gc.configure_tio = declare_parameter<bool>("gripper_configure_tio", false);
    gc.enable_tio_power = declare_parameter<bool>("gripper_enable_tio_power", true);
    gc.tio_voltage = declare_parameter<int>("gripper_tio_voltage", 0);
    gripper_ = std::make_unique<pgi140::Pgi140Gripper>(robot_, gc);
    auto dir = declare_parameter<std::string>("diagnostic_dir", "log/teleop");
    std::filesystem::create_directories(dir);
    auto suffix = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    csv_.open(std::filesystem::path(dir) / (arm_ + "_" + suffix + ".csv"));
    if (!csv_) throw std::runtime_error("cannot open telemetry CSV");
    csv_ << "kind,start_steady_s,end_steady_s,interval_s,ret,state,reason,pose_source_ros_s,pose_receive_s,"
      "joy_source_ros_s,joy_receive_s,tracking_receive_s,grip,tracked,mapped_x,mapped_y,mapped_z,"
      "target_x,target_y,target_z,raw_dx,raw_dy,raw_dz,dx,dy,dz,rx,ry,rz,"
      "planned_x,planned_y,planned_z,submitted_x,submitted_y,submitted_z,feedback_receive_s,"
      "feedback_x,feedback_y,feedback_z,j1_rad,j2_rad,j3_rad,j4_rad,j5_rad,j6_rad,"
      "hand_x,hand_y,hand_z,hand_qw,hand_qx,hand_qy,hand_qz,target_rx,target_ry,target_rz,"
      "submitted_rx,submitted_ry,submitted_rz,feedback_rx,feedback_ry,feedback_rz,fault_code,fault_time_s,dropped_records\n";
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(target, rclcpp::QoS(1),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { receive_pose(*m); });
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(buttons, rclcpp::QoS(1),
      [this](sensor_msgs::msg::Joy::SharedPtr m) { receive_joy(*m); });
    tracking_sub_ = create_subscription<std_msgs::msg::Bool>(tracking, rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mail_mutex_);
        mail_.tracked = m->data; mail_.tracking_receive = steady_seconds();
        if (!m->data) { mail_.interrupted = true; mail_.pose_valid = false; }
      });
    log_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { flush_records(); });
    RCLCPP_INFO(get_logger(), "[%s] SDK owner step=%d period=%.3fs watchdog=%.3fs; Trigger deferred until Grip release",
      arm_.c_str(), step_num_, cfg_.period_s, cfg_.max_interval_s);
    owner_ = std::thread([this]() { run(); });
  }
  ~RcControlNode() override { stop_ = true; if (owner_.joinable()) owner_.join(); flush_records(); }

private:
  struct Mailbox {
    jaka_teleop::ControllerPose pose;
    double pose_source = 0, pose_receive = -1, joy_source = 0, joy_receive = -1, tracking_receive = -1;
    bool tracked = false, grip = false, interrupted = false, pose_valid = false;
    double trigger = 0;
  } mail_;
  struct Record {
    std::string kind, reason;
    double start = 0, end = 0, interval = 0, feedback_time = -1;
    int ret = 0, state = 0;
    int fault_code = 0;
    double fault_time = 0;
    Mailbox input;
    jaka_teleop::Decision decision;
    CartesianPose target{}, planned{}, submitted{}, before{}, feedback{};
    JointValue joints{};
  };
  bool valid_source(double stamp) {
    const double age = now().seconds() - stamp;
    return stamp > 0 && std::isfinite(age) && age >= -.05 && age <= source_age_;
  }
  void receive_pose(const geometry_msgs::msg::PoseStamped &m) {
    if (faulted_) return;
    jaka_teleop::ControllerPose p;
    p.tran = {m.pose.position.x*1000., m.pose.position.y*1000., m.pose.position.z*1000.};
    Quaternion q{m.pose.orientation.w,m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z};
    double stamp = stamp_seconds(m.header.stamp);
    std::lock_guard<std::mutex> lock(mail_mutex_);
    if (stamp == mail_.pose_source) return; // cached frame is neither new nor an immediate fault
    if (!valid_source(stamp) || stamp < mail_.pose_source || !std::isfinite(p.tran.x) ||
        !std::isfinite(p.tran.y) || !std::isfinite(p.tran.z) || !jaka_teleop::quat_normalize(q,&p.quat)) {
      mail_.interrupted = true; mail_.pose_valid = false; return;
    }
    mail_.pose = p; mail_.pose_source = stamp; mail_.pose_receive = steady_seconds(); mail_.pose_valid = true;
  }
  void receive_joy(const sensor_msgs::msg::Joy &m) {
    if (faulted_) return;
    double stamp = stamp_seconds(m.header.stamp);
    std::lock_guard<std::mutex> lock(mail_mutex_);
    if (stamp == mail_.joy_source) return;
    if (!valid_source(stamp) || stamp < mail_.joy_source || m.axes.size() <= static_cast<size_t>(grip_axis_) ||
        !std::isfinite(m.axes[grip_axis_])) { mail_.interrupted = true; return; }
    bool grip = jaka_teleop::grip_state(mail_.grip,m.axes[grip_axis_],press_,release_);
    if (mail_.grip && !grip) mail_.interrupted = true; // sticky across an SDK stall
    mail_.grip = grip; mail_.joy_source = stamp; mail_.joy_receive = steady_seconds();
    double trigger = 0;
    if (linear_ && m.axes.size() > static_cast<size_t>(trigger_axis_)) trigger = m.axes[trigger_axis_];
    else if (!linear_ && m.buttons.size() > static_cast<size_t>(trigger_button_)) trigger = m.buttons[trigger_button_] ? 1. : 0.;
    if (std::isfinite(trigger)) mail_.trigger = std::clamp(trigger,0.,1.);
  }
  Mailbox snapshot() {
    std::lock_guard<std::mutex> lock(mail_mutex_);
    Mailbox copy = mail_; mail_.interrupted = false; return copy;
  }
  void record(Record r) {
    r.state = static_cast<int>(control_->state()); r.reason = control_->reason();
    r.fault_code = control_->fault_code(); r.fault_time = control_->fault_time();
    r.target = control_->target(); r.planned = control_->planned(); r.submitted = control_->submitted();
    r.feedback = feedback_; r.feedback_time = feedback_time_; r.joints = joints_;
    std::lock_guard<std::mutex> lock(record_mutex_);
    if (records_.size() == 4096) { records_.pop_front(); ++dropped_; }
    records_.push_back(std::move(r));
  }
  void flush_records() {
    std::deque<Record> copy; unsigned long dropped;
    { std::lock_guard<std::mutex> lock(record_mutex_); copy.swap(records_); dropped = dropped_; }
    csv_ << std::setprecision(12);
    for (const auto &r : copy) {
      const auto &m=r.input; const auto &d=r.decision;
      csv_ << r.kind << ',' << r.start << ',' << r.end << ',' << r.interval << ',' << r.ret << ',' << r.state << ',' << r.reason
        << ',' << m.pose_source << ',' << m.pose_receive << ',' << m.joy_source << ',' << m.joy_receive
        << ',' << m.tracking_receive << ',' << m.grip << ',' << m.tracked
        << ',' << d.raw_translation.x << ',' << d.raw_translation.y << ',' << d.raw_translation.z
        << ',' << d.target_translation.x << ',' << d.target_translation.y << ',' << d.target_translation.z
        << ',' << d.target_translation.x-r.before.tran.x << ',' << d.target_translation.y-r.before.tran.y << ',' << d.target_translation.z-r.before.tran.z
        << ',' << d.increment.tran.x << ',' << d.increment.tran.y << ',' << d.increment.tran.z
        << ',' << d.increment.rpy.rx << ',' << d.increment.rpy.ry << ',' << d.increment.rpy.rz
        << ',' << r.planned.tran.x << ',' << r.planned.tran.y << ',' << r.planned.tran.z
        << ',' << r.submitted.tran.x << ',' << r.submitted.tran.y << ',' << r.submitted.tran.z
        << ',' << r.feedback_time << ',' << r.feedback.tran.x << ',' << r.feedback.tran.y << ',' << r.feedback.tran.z;
      for (double j : r.joints.jVal) csv_ << ',' << j;
      csv_ << ',' << m.pose.tran.x << ',' << m.pose.tran.y << ',' << m.pose.tran.z
        << ',' << m.pose.quat.s << ',' << m.pose.quat.x << ',' << m.pose.quat.y << ',' << m.pose.quat.z
        << ',' << r.target.rpy.rx << ',' << r.target.rpy.ry << ',' << r.target.rpy.rz
        << ',' << r.submitted.rpy.rx << ',' << r.submitted.rpy.ry << ',' << r.submitted.rpy.rz
        << ',' << r.feedback.rpy.rx << ',' << r.feedback.rpy.ry << ',' << r.feedback.rpy.rz
        << ',' << r.fault_code << ',' << r.fault_time << ',' << dropped << '\n';
    }
    csv_.flush();
  }
  bool check(int ret,const char *context) {
    RCLCPP_INFO(get_logger(), "[%s] %s ret=%d",arm_.c_str(),context,ret);
    if (ret == 0) return true;
    control_->fault(ret,steady_seconds(),context); return false;
  }
  void read_configuration() {
    char version[256]{}; int user=-1,tool=-1,collision=-1;
    CartesianPose uf{},tcp{}; Quaternion mount{}; Rpy mr{}; PayLoad load{};
    int rv=robot_.get_sdk_version(version), ru=robot_.get_user_frame_id(&user), rt=robot_.get_tool_id(&tool);
    int ruf=ru==0 ? robot_.get_user_frame_data(user,&uf) : ru;
    int rtf=rt==0 ? robot_.get_tool_data(tool,&tcp) : rt;
    int rm=robot_.get_installation_angle(&mount,&mr), rl=robot_.get_payload(&load), rc=robot_.get_collision_level(&collision);
    if (ru==0 && rt==0) { user_id_=static_cast<unsigned int>(user); tool_id_=static_cast<unsigned int>(tool); frame_valid_=true; }
    RCLCPP_INFO(get_logger(), "[%s] SDK=%s ret=%d user=%d ret=%d TCP=%d ret=%d collision=%d ret=%d (preserved)",
      arm_.c_str(),version,rv,user,ru,tool,rt,collision,rc);
    RCLCPP_INFO(get_logger(), "[%s] user_frame=(%.3f %.3f %.3f; %.6f %.6f %.6f) ret=%d TCP=(%.3f %.3f %.3f; %.6f %.6f %.6f) ret=%d",
      arm_.c_str(),uf.tran.x,uf.tran.y,uf.tran.z,uf.rpy.rx,uf.rpy.ry,uf.rpy.rz,ruf,tcp.tran.x,tcp.tran.y,tcp.tran.z,tcp.rpy.rx,tcp.rpy.ry,tcp.rpy.rz,rtf);
    RCLCPP_INFO(get_logger(), "[%s] mount_q=(%.6f %.6f %.6f %.6f) rpy=(%.6f %.6f %.6f) ret=%d payload=%.3fkg centroid=(%.3f %.3f %.3f)mm ret=%d",
      arm_.c_str(),mount.s,mount.x,mount.y,mount.z,mr.rx,mr.ry,mr.rz,rm,load.mass,load.centroid.x,load.centroid.y,load.centroid.z,rl);
  }
  bool startup() {
    if (stopping()) return false;
    if (!check(robot_.login_in(ip_.c_str()),"login_in")) return false;
    connected_=true; read_configuration();
    if (stopping()) return false;
    if (!check(robot_.set_status_data_update_time_interval(100),"status_interval_100ms") ||
        !check(robot_.set_block_wait_timeout(120),"block_wait_timeout")) return false;
    RobotStatus_simple s{};
    if (!check(robot_.get_robot_status_simple(&s),"startup_status")) return false;
    if (s.errcode) { control_->fault(s.errcode,steady_seconds(),"startup_controller_error"); return false; }
    RobotStatus protection{};
    if (!check(robot_.get_robot_status(&protection),"startup_protection")) return false;
    if (protection.errcode || protection.protective_stop || protection.emergency_stop) {
      control_->fault(protection.errcode ? protection.errcode : kFeedbackFault,steady_seconds(),"startup_protection"); return false;
    }
    if (stopping()) return false;
    std::string error;
    if (!gripper_->initialize(&error)) {
      RCLCPP_ERROR(get_logger(), "[%s] gripper startup: %s",arm_.c_str(),error.c_str());
      control_->fault(kGripperFault,steady_seconds(),"gripper_initialization"); return false;
    }
    if (stopping()) return false;
    if (!s.powered_on) {
      if (!check(robot_.power_on(),"startup_power_on") || !startup_wait(8.)) return false;
    }
    if (stopping()) return false;
    if (!s.enabled) {
      if (!check(robot_.enable_robot(),"startup_enable_robot") || !startup_wait(4.)) return false;
    }
    if (stopping()) return false;
    int ret= filter_=="none" ? robot_.servo_move_use_none_filter() : filter_=="speed_foresight" ?
      robot_.servo_speed_foresight(foresight_buf_,foresight_kp_) : robot_.servo_move_use_carte_NLF(nlf_v_,nlf_a_,nlf_j_,nlf_rv_,nlf_ra_,nlf_rj_);
    RCLCPP_INFO(get_logger(), "[%s] filter=%s nlf=(%.3f %.3f %.3f mm/s,s2,s3; %.3f %.3f %.3f deg/s,s2,s3) foresight=(%d %.3f) ret=%d",
      arm_.c_str(),filter_.c_str(),nlf_v_,nlf_a_,nlf_j_,nlf_rv_,nlf_ra_,nlf_rj_,foresight_buf_,foresight_kp_,ret);
    if (!check(ret,filter_.c_str()) || stopping() || !check(robot_.servo_move_enable(true),"servo_move_enable")) return false;
    servo_enabled_=true; return true;
  }
  void poll_feedback(const Mailbox &m) {
    Record r; r.kind="feedback"; r.input=m; r.start=steady_seconds(); RobotStatus s{};
    r.ret=robot_.get_robot_status(&s); r.end=steady_seconds();
    if (r.ret) control_->fault(r.ret,r.end,"feedback_read");
    else if (s.errcode || s.protective_stop || s.emergency_stop || !s.enabled || !s.powered_on)
      control_->fault(s.errcode ? s.errcode : kFeedbackFault,r.end,"controller_protection_or_disabled");
    else {
      feedback_.tran={s.cartesiantran_position[0],s.cartesiantran_position[1],s.cartesiantran_position[2]};
      feedback_.rpy={s.cartesiantran_position[3],s.cartesiantran_position[4],s.cartesiantran_position[5]};
      for (int i=0;i<6;++i) joints_.jVal[i]=s.joint_position[i];
      feedback_time_=r.end; control_->set_feedback(feedback_,feedback_time_,true);
      if (frame_valid_ && (user_id_!=s.current_user_id || tool_id_!=s.current_tool_id))
        control_->fault(kFeedbackFault,r.end,"active_user_or_tcp_changed");
      user_id_=s.current_user_id; tool_id_=s.current_tool_id; frame_valid_=true;
    }
    if (r.end-r.start > cfg_.max_interval_s) control_->fault(kTimingFault,r.end,"feedback_call_overrun");
    record(r);
  }
  void service_gripper() {
    // No timeout/asynchronous TIO API is supplied. Coalesce pending Trigger,
    // wait for Grip release, then exit servo before the blocking SDK operation.
    control_->external_pause("gripper_maintenance");
    if (!check(robot_.servo_move_enable(false),"gripper_servo_exit")) return;
    servo_enabled_=false;
    auto fresh=snapshot();
    if (!release_ready(fresh) || stopping()) {
      if (!stopping() && check(robot_.servo_move_enable(true),"gripper_cancelled_servo_resume")) servo_enabled_=true;
      return;
    }
    Record r; r.kind="gripper"; r.input=fresh; r.start=steady_seconds(); std::string error;
    bool ok=linear_ ? gripper_->Grip(static_cast<float>(fresh.trigger),&error) : gripper_->set_closed(fresh.trigger>.5,&error);
    r.end=steady_seconds(); r.ret=ok ? 0 : gripper_->last_sdk_return();
    if (!ok || r.end-r.start > gripper_timeout_) {
      control_->fault(ok ? kTimingFault : (r.ret ? r.ret : kGripperFault),r.end,ok ? "gripper_call_overrun" : "gripper_call_failed");
      RCLCPP_ERROR(get_logger(), "[%s] gripper duration=%.6fs detail=%s",arm_.c_str(),r.end-r.start,error.c_str());
    } else {
      applied_trigger_=fresh.trigger;
      if (!stopping() && check(robot_.servo_move_enable(true),"gripper_servo_resume")) servo_enabled_=true;
    }
    record(r);
  }
  void run() {
    try {
      if (startup()) {
        double next_feedback=0,last_start=0; auto previous=control_->state();
        auto deadline=Clock::now();
        while (!stop_ && rclcpp::ok()) {
          if (Clock::now()<deadline) std::this_thread::sleep_until(deadline);
          if (stop_ || !rclcpp::ok()) break;
          double t=steady_seconds(); auto m=snapshot();
          apply_input(m);
          if (t>=next_feedback && control_->state()!=jaka_teleop::State::Fault) {
            poll_feedback(m); next_feedback=steady_seconds()+feedback_period_;
            m=snapshot(); apply_input(m);
          }
          if (stopping()) break;
          if (control_->state()!=jaka_teleop::State::Fault && release_ready(m) &&
              std::fabs(m.trigger-applied_trigger_)>=trigger_deadband_) {
            service_gripper(); m=snapshot(); apply_input(m);
          }
          if (stopping()) break;
          auto plan_clock=Clock::now();
          auto d=control_->plan(std::chrono::duration<double>(plan_clock.time_since_epoch()).count());
          deadline=plan_clock+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cfg_.period_s));
          if (d.send && servo_enabled_) {
            Record r; r.kind="servo"; r.input=m; r.decision=d; r.before=control_->submitted();
            r.start=steady_seconds(); r.interval=last_start>0 ? r.start-last_start : 0; last_start=r.start;
            // Sole servo_p call site. Each plan constructs a fresh zero on hold.
            r.ret=robot_.servo_p(&d.increment,MoveMode::INCR,static_cast<unsigned int>(step_num_));
            r.end=steady_seconds(); control_->commit(r.ret,r.end);
            if (r.end-r.start>cfg_.max_interval_s) control_->fault(kTimingFault,r.end,"servo_call_overrun");
            record(r);
          }
          if (control_->state()!=previous) {
            RCLCPP_WARN(get_logger(), "[%s] state %d -> %d reason=%s fault=%d at=%.6f",arm_.c_str(),
              static_cast<int>(previous),static_cast<int>(control_->state()),control_->reason().c_str(),control_->fault_code(),control_->fault_time());
            previous=control_->state();
          }
          if (control_->state()==jaka_teleop::State::Fault) break;
          // Deadline is tied to plan/send, not the earlier loop start. Blocking
          // calls consume it, so no extra sleep or missed-cycle replay is added.
        }
      }
    } catch (const std::exception &e) {
      control_->fault(kFeedbackFault,steady_seconds(),"owner_exception");
      RCLCPP_ERROR(get_logger(), "[%s] SDK owner exception: %s",arm_.c_str(),e.what());
    }
    // All teardown belongs to the same SDK owner, even on partial startup.
    // Do not auto-open the gripper on fault/shutdown: it may hold a payload.
    faulted_=control_->state()==jaka_teleop::State::Fault;
    if (faulted_) RCLCPP_ERROR(get_logger(), "[%s] latched fault=%d time=%.6f context=%s",arm_.c_str(),control_->fault_code(),control_->fault_time(),control_->reason().c_str());
    if (connected_) {
      if (servo_enabled_) { int ret=robot_.servo_move_enable(false); RCLCPP_WARN(get_logger(), "[%s] servo exit ret=%d",arm_.c_str(),ret); }
      int ret=robot_.login_out(); RCLCPP_INFO(get_logger(), "[%s] logout ret=%d",arm_.c_str(),ret);
    }
    Record r; r.kind="owner_stopped"; r.start=r.end=steady_seconds(); record(r);
  }
  JAKAZuRobot robot_;
  std::unique_ptr<pgi140::Pgi140Gripper> gripper_;
  jaka_teleop::Config cfg_;
  std::unique_ptr<jaka_teleop::TeleopControl> control_;
  void apply_input(const Mailbox &m) {
    if (m.interrupted) control_->external_pause("input_interrupted");
    const bool tracked=input_tracked(m);
    control_->set_pose(m.pose,m.pose_receive,tracked);
    // Bridge sends a synthetic Grip=0 on loss. Only a fresh, matched tracked
    // pose/Joy frame can acknowledge release; recovery with Grip still held
    // must not silently re-engage using that synthetic release.
    if (m.grip || release_ready(m))
      control_->set_grip(m.grip,m.joy_receive);
  }
  bool input_tracked(const Mailbox &m) const {
    return m.pose_valid && (!require_tracking_ || (m.tracked && steady_seconds()-m.tracking_receive<=cfg_.input_timeout_s));
  }
  bool release_ready(const Mailbox &m) const {
    const double t=steady_seconds();
    return !m.grip && input_tracked(m) && m.pose_source==m.joy_source &&
      t-m.pose_receive<=cfg_.input_timeout_s && t-m.joy_receive<=cfg_.input_timeout_s;
  }
  bool stopping() const { return stop_ || !rclcpp::ok(); }
  bool startup_wait(double seconds) const {
    const double end=steady_seconds()+seconds;
    while (!stopping() && steady_seconds()<end) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return !stopping();
  }
  std::thread owner_; std::atomic<bool> stop_{false},faulted_{false};
  std::mutex mail_mutex_,record_mutex_; std::deque<Record> records_; unsigned long dropped_=0;
  std::ofstream csv_; std::string arm_,ip_,filter_;
  int grip_axis_,trigger_axis_,trigger_button_,step_num_,foresight_buf_;
  double press_,release_,trigger_deadband_,feedback_period_,gripper_timeout_,source_age_;
  double nlf_v_,nlf_a_,nlf_j_,nlf_rv_,nlf_ra_,nlf_rj_,foresight_kp_;
  bool linear_,require_tracking_,connected_=false,servo_enabled_=false,frame_valid_=false;
  unsigned int user_id_=0,tool_id_=0; double applied_trigger_=0,feedback_time_=-1;
  CartesianPose feedback_{}; JointValue joints_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_sub_;
  rclcpp::TimerBase::SharedPtr log_timer_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv); auto node=std::make_shared<RcControlNode>();
  rclcpp::spin(node); node.reset(); rclcpp::shutdown(); return 0;
}
