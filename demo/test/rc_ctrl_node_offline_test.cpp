// Executes the production owner loop with an in-memory ROS mailbox and a fake
// implementation of the bundled SDK declarations. No vendor library is linked.
#include <cassert>
#include <cstring>
#include <functional>
#include <iostream>
#include <vector>
#include "std_msgs/msg/int64.hpp"
#define main xrtoolkit_production_main
#include "../src/jaka_driver/src/rc_ctrl_node.cpp"
#undef main

namespace fake {
struct Event {
  std::string name;
  double time;
  std::thread::id thread;
  CartesianPose increment{};
};
std::mutex mutex;
std::vector<Event> events;
CartesianPose pose{};
int servo_return = 0, status_return = 0, protective_error = 0;
int servo_delay_ms = 0, tio_delay_ms = 0;
bool servo_enabled = false;
bool state_tracked=false;
int64_t state_stamp_ns=0;
void tracking(bool valid,double source);
void add(const std::string & name, CartesianPose p = {}) {
  std::lock_guard<std::mutex> lock(mutex);
  events.push_back({name,steady_seconds(),std::this_thread::get_id(),p});
}
std::vector<Event> snapshot() {
  std::lock_guard<std::mutex> lock(mutex); return events;
}
std::size_t count(const std::string & name) {
  const auto copy=snapshot();
  return std::count_if(copy.begin(),copy.end(),[&](const Event &e){return e.name==name;});
}
void reset() {
  std::lock_guard<std::mutex> lock(mutex);
  events.clear(); pose={}; servo_return=status_return=protective_error=0;
  servo_delay_ms=tio_delay_ms=0; servo_enabled=false;
  state_tracked=false;state_stamp_ns=0;
}
void set(const std::function<void()> & change) {
  std::lock_guard<std::mutex> lock(mutex); change();
}
bool wait(const std::function<bool()> & condition, int timeout_ms=1000) {
  const double end=steady_seconds()+timeout_ms*.001;
  while(steady_seconds()<end) {
    if(condition()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return condition();
}
bool zero(const CartesianPose &p) {
  return std::abs(p.tran.x)+std::abs(p.tran.y)+std::abs(p.tran.z)+
    std::abs(p.rpy.rx)+std::abs(p.rpy.ry)+std::abs(p.rpy.rz)<1e-12;
}
void input(bool grip, double x_mm=0, double trigger=0, bool tracked=true, double source=0) {
  const double ros=source>0 ? source : rclcpp::Time{}.seconds();
  geometry_msgs::msg::PoseStamped p;
  p.header.stamp.sec=static_cast<int32_t>(ros);
  p.header.stamp.nanosec=static_cast<uint32_t>((ros-p.header.stamp.sec)*1e9);
  p.pose.position.x=x_mm*.001; p.pose.orientation.w=1;
  sensor_msgs::msg::Joy j;
  j.header=p.header; j.buttons=std::vector<int32_t>(16,1); j.axes={grip ? 1.f : 0.f,0.f,static_cast<float>(trigger),0.f}; j.buttons=std::vector<int32_t>(16,1);
  tracking(tracked,ros);
  rclcpp::offline_publish("/rc_ctrl/left_target",p);
  rclcpp::offline_publish("/rc_ctrl/button",j);
}
void pump(int ms,bool grip,double x=0,double trigger=0,bool tracked=true) {
  const double end=steady_seconds()+ms*.001;
  while(steady_seconds()<end) {
    input(grip,x,trigger,tracked);
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}
// Model unordered ROS topics during loss/reacquisition: heartbeat and Joy can
// arrive before the next valid pose. They must not revive the previous pose.
void pump_without_pose(int ms,bool grip,bool tracked,double trigger=0) {
  const double end=steady_seconds()+ms*.001;
  while(steady_seconds()<end) {
    const double ros=rclcpp::Time{}.seconds();
    sensor_msgs::msg::Joy j;
    j.header.stamp.sec=static_cast<int32_t>(ros);
    j.header.stamp.nanosec=static_cast<uint32_t>((ros-j.header.stamp.sec)*1e9);
    j.axes={grip ? 1.f : 0.f,0.f,static_cast<float>(trigger),0.f}; j.buttons=std::vector<int32_t>(16,1);
    tracking(tracked,ros);
    rclcpp::offline_publish("/rc_ctrl/button",j);
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}
void pump_without_joy(int ms,double x_mm) {
  const double end=steady_seconds()+ms*.001;
  while(steady_seconds()<end) {
    const double ros=rclcpp::Time{}.seconds();
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp.sec=static_cast<int32_t>(ros);
    p.header.stamp.nanosec=static_cast<uint32_t>((ros-p.header.stamp.sec)*1e9);
    p.pose.position.x=x_mm*.001;p.pose.orientation.w=1;
    tracking(true,ros);
    rclcpp::offline_publish("/rc_ctrl/left_target",p);
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}
std::shared_ptr<RcControlNode> start(double gripper_timeout=.03) {
  reset(); rclcpp::offline_parameters.clear();
  rclcpp::offline_parameters["gripper_initialize"]=false;
  rclcpp::offline_parameters["max_servo_interval_s"]=.12;
  rclcpp::offline_parameters["feedback_period_s"]=.03;
  rclcpp::offline_parameters["gripper_call_timeout_s"]=gripper_timeout;
  rclcpp::offline_parameters["gripper_trigger_deadband"]=.02;
  rclcpp::offline_parameters["diagnostic_dir"]=std::string("build/offline/integration_logs");
  auto node=std::make_shared<RcControlNode>();
  assert(wait([]{return count("servo")>=3;}));
  return node;
}
void assert_owner() {
  const auto copy=snapshot(); assert(!copy.empty());
  for(const auto &e:copy) assert(e.thread==copy.front().thread);
  assert(copy.front().thread!=std::this_thread::get_id());
  assert(count("collision_recover")==0 && count("clear_error")==0);
}
void assert_no_grip_recovery() {
  const auto n=snapshot().size();
  pump(40,true,200); pump(20,false); pump(40,true,400);
  assert(snapshot().size()==n);
  assert(count("power_on")==0 && count("enable_robot")==0);
  assert(count("collision_recover")==0 && count("clear_error")==0);
}
}  // namespace fake

JAKAZuRobot::JAKAZuRobot() {}
JAKAZuRobot::~JAKAZuRobot() {}
errno_t JAKAZuRobot::login_in(const char *,bool) {fake::add("login");return 0;}
errno_t JAKAZuRobot::login_out() {fake::add("logout");return 0;}
errno_t JAKAZuRobot::power_on() {fake::add("power_on");return 0;}
errno_t JAKAZuRobot::enable_robot() {fake::add("enable_robot");return 0;}
errno_t JAKAZuRobot::clear_error() {fake::add("clear_error");return 0;}
errno_t JAKAZuRobot::collision_recover() {fake::add("collision_recover");return 0;}
errno_t JAKAZuRobot::set_status_data_update_time_interval(float) {fake::add("status_interval");return ERR_OBSOLETE;}
errno_t JAKAZuRobot::set_block_wait_timeout(float) {fake::add("block_timeout");return ERR_OBSOLETE;}
errno_t JAKAZuRobot::get_sdk_version(char *v) {fake::add("version");std::strcpy(v,"OFFLINE_FAKE");return 0;}
errno_t JAKAZuRobot::get_user_frame_id(int *v) {fake::add("user_id");*v=15;return 0;}
errno_t JAKAZuRobot::get_tool_id(int *v) {fake::add("tool_id");*v=15;return 0;}
errno_t JAKAZuRobot::get_user_frame_data(int,CartesianPose *p) {fake::add("user_data");*p={};return 0;}
errno_t JAKAZuRobot::get_tool_data(int,CartesianPose *p) {fake::add("tool_data");*p={};return 0;}
errno_t JAKAZuRobot::get_installation_angle(Quaternion *q,Rpy *r) {fake::add("installation");*q={1,0,0,0};*r={};return 0;}
errno_t JAKAZuRobot::get_payload(PayLoad *p) {fake::add("payload");*p={};return 0;}
errno_t JAKAZuRobot::get_collision_level(int *v) {fake::add("collision_level");*v=1;return 0;}
errno_t JAKAZuRobot::is_in_servomove(BOOL *v) {fake::add("in_servo");std::lock_guard<std::mutex> l(fake::mutex);*v=fake::servo_enabled;return 0;}
errno_t JAKAZuRobot::get_robot_status_simple(RobotStatus_simple *s) {
  fake::add("simple_status");std::lock_guard<std::mutex> l(fake::mutex);
  *s={};s->powered_on=1;s->enabled=1;return fake::status_return;
}
errno_t JAKAZuRobot::get_robot_status(RobotStatus *s) {
  fake::add("feedback");std::lock_guard<std::mutex> l(fake::mutex);*s={};
  s->enabled=s->powered_on=s->is_socket_connect=1;
  s->current_tool_id=s->current_user_id=15;s->errcode=fake::protective_error;
  s->protective_stop=fake::protective_error ? 1 : 0;
  s->cartesiantran_position[0]=fake::pose.tran.x;
  s->cartesiantran_position[1]=fake::pose.tran.y;
  s->cartesiantran_position[2]=fake::pose.tran.z;
  return 0;
}
errno_t JAKAZuRobot::servo_move_use_none_filter() {fake::add("none_filter");return 0;}
errno_t JAKAZuRobot::servo_speed_foresight(int,double) {fake::add("foresight");return 0;}
errno_t JAKAZuRobot::servo_move_use_carte_NLF(double,double,double,double,double,double) {fake::add("nlf");return 0;}
errno_t JAKAZuRobot::servo_move_enable(BOOL v) {
  fake::add(v ? "servo_enable" : "servo_disable");
  std::lock_guard<std::mutex> l(fake::mutex);fake::servo_enabled=v!=0;return 0;
}
errno_t JAKAZuRobot::servo_p(const CartesianPose *p,MoveMode mode,unsigned int step) {
  assert(mode==MoveMode::INCR && step==1);fake::add("servo",*p);
  int delay,ret;
  {std::lock_guard<std::mutex> l(fake::mutex);delay=fake::servo_delay_ms;ret=fake::servo_return;assert(fake::servo_enabled);}
  if(delay) std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  if(!ret) {std::lock_guard<std::mutex> l(fake::mutex);fake::pose.tran.x+=p->tran.x;fake::pose.tran.y+=p->tran.y;fake::pose.tran.z+=p->tran.z;}
  return ret;
}
errno_t JAKAZuRobot::set_tio_vout_param(int,int) {fake::add("tio_power");return 0;}
errno_t JAKAZuRobot::set_tio_pin_mode(int,int) {fake::add("tio_pin");return 0;}
errno_t JAKAZuRobot::set_rs485_chn_mode(int,int) {fake::add("rs485_mode");return 0;}
errno_t JAKAZuRobot::set_rs485_chn_comm(ModRtuComm) {fake::add("rs485_comm");return 0;}
errno_t JAKAZuRobot::send_tio_rs_command(int,uint8_t *frame,int length) {
  assert(length==8); const int address=(frame[2]<<8)|frame[3];
  CartesianPose decoded{};decoded.tran.x=address;decoded.tran.y=(frame[4]<<8)|frame[5];
  fake::add(address==0x0103 ? "tio_position" : "tio_config",decoded);int delay;
  {std::lock_guard<std::mutex> l(fake::mutex);assert(!fake::servo_enabled);delay=fake::tio_delay_ms;}
  if(delay) std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  return 0;
}

void regression_source_deadline() {
  using namespace fake;
  auto node=start();
  auto delayed=[](int ms,bool grip,double x) {
    const double end=steady_seconds()+ms*.001;
    while(steady_seconds()<end) {
      input(grip,x,0,true,rclcpp::Time{}.seconds()-.15);
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
  };
  delayed(24,false,0); delayed(24,true,0); delayed(32,true,200);
  const double frozen=steady_seconds();
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  bool nonzero_after_expiry=false;
  for(const auto &e:snapshot()) if(e.name=="servo" && e.time>frozen+.065 && !zero(e.increment)) nonzero_after_expiry=true;
  node.reset();
  if(nonzero_after_expiry) throw std::runtime_error("source 200ms deadline bypassed by receive-only TTL");
  std::cout<<"PASS source deadline expires delayed150ms messages before receive200ms TTL\n";
}
void regression_interleaved_pairing() {
  using namespace fake;
  auto node=start(); pump(24,false); pump(24,true);
  const auto before=count("tio_position");
  const double base=rclcpp::Time{}.seconds();
  auto pose=[](double stamp) {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp.sec=static_cast<int32_t>(stamp);
    p.header.stamp.nanosec=static_cast<uint32_t>((stamp-p.header.stamp.sec)*1e9);
    p.pose.orientation.w=1;
    rclcpp::offline_publish("/rc_ctrl/left_target",p);
  };
  pose(base);
  for(int i=0;i<24;++i) {
    const double stamp=base+i*.004;
    pose(base+(i+1)*.004); // latest Pose is always one ahead of Joy
    sensor_msgs::msg::Joy j;
    j.header.stamp.sec=static_cast<int32_t>(stamp);
    j.header.stamp.nanosec=static_cast<uint32_t>((stamp-j.header.stamp.sec)*1e9);
    j.axes={0,0,.8f,0}; j.buttons=std::vector<int32_t>(16,1);
    tracking(true,stamp);
    rclcpp::offline_publish("/rc_ctrl/button",j);
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
  const bool handled=count("tio_position")>before;
  node.reset();
  if(!handled) throw std::runtime_error("complete release frames permanently lost to latest Pose(k+1)/Joy(k)");
  std::cout<<"PASS complete interleaved release frames are consumed\n";
}
void regression_invalid_reads_and_restart() {
  using namespace fake;
  {
    auto node=start();pump(48,false,0,.8);
    assert(count("tio_position")==0); // initial nonzero Trigger is only a baseline
    pump(48,false,0,.7);assert(count("tio_position")==1);
    bool decoded=false;
    for(const auto &e:snapshot()) if(e.name=="tio_position") decoded=e.increment.tran.y==300;
    assert(decoded);node.reset();
  }
  for(int scenario=0;scenario<6;++scenario) {
    auto node=start();pump(24,false);pump(24,true);pump(32,true,200,.8);
    const auto count_before=count("tio_position");
    const double end=steady_seconds()+.032;
    while(steady_seconds()<end) {
      const double ros=rclcpp::Time{}.seconds();
      geometry_msgs::msg::PoseStamped p;p.pose.orientation.w=1;
      p.header.stamp.sec=static_cast<int32_t>(ros);
      p.header.stamp.nanosec=static_cast<uint32_t>((ros-p.header.stamp.sec)*1e9);
      sensor_msgs::msg::Joy j;j.header=p.header;j.axes={0,0,.8f,0};
      j.buttons=std::vector<int32_t>(16,1);
      if(scenario==0) j.buttons[12]=0; // exception represented independently
      if(scenario==1) j.axes[0]=std::numeric_limits<float>::quiet_NaN();
      if(scenario==2) j.axes[0]=-.01f;
      if(scenario==3) j.axes[0]=1.01f;
      if(scenario==4) j.buttons[14]=0; // failed Trigger cannot authorize release/TIO
      if(scenario==5) j.buttons.clear(); // old publisher missing validity metadata
      tracking(true,ros);
      rclcpp::offline_publish("/rc_ctrl/left_target",p);
      rclcpp::offline_publish("/rc_ctrl/button",j);
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
    assert(count("tio_position")==count_before);
    const auto after=snapshot().size();pump(48,true,400,.8);
    const auto copy=snapshot();
    for(std::size_t i=after;i<copy.size();++i) if(copy[i].name=="servo") assert(zero(copy[i].increment));
    assert(count("tio_position")==count_before);
    pump(48,false,400,.8);assert(count("tio_position")==count_before+1);
    node.reset();assert_owner();
  }
  std::cout<<"PASS invalid Grip/Trigger reads never release or run TIO; held recovery waits; startup Trigger only seeds baseline\n";
}
void regression_gripper_deadline() {
  using namespace fake;
  auto node=start();
  for(int i=0;i<6;++i) {input(false,0,0,true,rclcpp::Time{}.seconds()-.15);std::this_thread::sleep_for(std::chrono::milliseconds(4));}
  set([]{tio_delay_ms=80;});
  input(false,0,.8,true,rclcpp::Time{}.seconds()-.15);
  assert(wait([]{return count("logout")==1;}));
  const auto positions=count("tio_position");node.reset();
  if(positions) throw std::runtime_error("position register written after source expired during force/speed SDK calls");
  std::cout<<"PASS source deadline rechecked between lazy force/speed/position writes\n";
}
namespace fake {
struct InputBundle {
  geometry_msgs::msg::PoseStamped pose;
  sensor_msgs::msg::Joy joy;
};
InputBundle bundle(double source,bool grip,double x_mm,double trigger,bool valid=true) {
  InputBundle b;
  b.pose.header.stamp.sec=static_cast<int32_t>(source);
  b.pose.header.stamp.nanosec=static_cast<uint32_t>((source-b.pose.header.stamp.sec)*1e9);
  b.pose.pose.position.x=x_mm*.001;b.pose.pose.orientation.w=1;
  b.joy.header=b.pose.header;b.joy.axes={grip ? 1.f : 0.f,0.f,static_cast<float>(trigger),0.f};
  b.joy.buttons=std::vector<int32_t>(16,1);
  if(!valid) {b.joy.buttons[12]=0;b.joy.buttons[14]=0;}
  return b;
}
void tracking(bool valid,double source) {
  std_msgs::msg::Bool t;t.data=valid;
  rclcpp::offline_publish("/rc_ctrl/left_tracking",t);
  if(!state_stamp_ns || state_tracked!=valid) {
    const int64_t sec=static_cast<int64_t>(source);
    state_stamp_ns=sec*1000000000LL+static_cast<int64_t>((source-sec)*1e9);
    state_tracked=valid;
  }
  std_msgs::msg::Int64 state;state.data=valid ? state_stamp_ns : -state_stamp_ns;
  rclcpp::offline_publish("/rc_ctrl/left_input_state",state);
}
void deliver(const InputBundle &b) {
  rclcpp::offline_publish("/rc_ctrl/left_target",b.pose);
  rclcpp::offline_publish("/rc_ctrl/button",b.joy);
}
}
void regression_inflight_epoch() {
  using namespace fake;
  bool failure=false;
  for(bool old_trigger:{false,true}) {
    auto node=start(.25);pump(24,false);pump(24,true);
    const double base=rclcpp::Time{}.seconds();
    const auto released=bundle(base,false,0,old_trigger ? .8 : 0);
    const auto pressed=bundle(base+.001,true,0,old_trigger ? .8 : 0);
    const auto moved=bundle(base+.002,true,200,old_trigger ? .8 : 0);
    const auto invalid_loss=bundle(base+.003,false,200,old_trigger ? .8 : 0,false);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const double recovery=rclcpp::Time{}.seconds();
    const auto recovery_held=bundle(recovery,true,500,old_trigger ? .8 : 0);
    tracking(false,base+.003);tracking(true,recovery);
    const auto after_loss=snapshot().size();const auto position_before=count("tio_position");
    // Source topic order is release, press, movement, invalid loss, recovered
    // held. Every Pose and Joy is delivered in that order; only tracking passes
    // the old complete frames on its independent topic.
    deliver(released);std::this_thread::sleep_for(std::chrono::milliseconds(16));
    deliver(pressed);std::this_thread::sleep_for(std::chrono::milliseconds(16));
    deliver(moved);std::this_thread::sleep_for(std::chrono::milliseconds(32));
    rclcpp::offline_publish("/rc_ctrl/button",invalid_loss.joy);
    deliver(recovery_held);pump(32,true,500,old_trigger ? .8 : 0);
    bool resumed=false;
    auto copy=snapshot();
    for(std::size_t i=after_loss;i<copy.size();++i)
      if(copy[i].name=="servo" && !zero(copy[i].increment)) resumed=true;
    const auto old_positions=count("tio_position")-position_before;
    std::cout<<"INFLIGHT old_trigger="<<old_trigger<<" nonzero_servo="<<resumed
      <<" old_position_writes="<<old_positions<<'\n';
    failure=failure || resumed || old_positions!=0;
    // A genuinely new post-recovery release/repress must still work.
    pump(32,false,500,old_trigger ? .8 : 0);pump(24,true,500,old_trigger ? .8 : 0);
    const auto before_new=snapshot().size();pump(64,true,510,old_trigger ? .8 : 0);
    copy=snapshot();bool new_motion=false;
    for(std::size_t i=before_new;i<copy.size();++i)
      if(copy[i].name=="servo" && !zero(copy[i].increment)) new_motion=true;
    if(!new_motion) failure=true;
    node.reset();assert_owner();
  }
  if(failure) throw std::runtime_error("pre-loss unseen complete frames crossed tracking recovery without a new operator release");
  std::cout<<"PASS in-flight pre-loss release/press/move isolated; held recovery waits; new release/repress works\n";
}
void regression_trigger_request_latch() {
  using namespace fake;
  auto node=start(.25);pump(24,false);
  // Establish the old global revision baseline at .81, while the actual
  // release request is .799 (only .011 away from that unrelated baseline).
  pump(24,true,0,.81);pump(24,true,0,.799);
  set([]{tio_delay_ms=35;});
  const auto request=bundle(rclcpp::Time{}.seconds(),false,0,.799);
  deliver(request);
  assert(wait([]{return count("tio_config")>=1;}));
  const auto excursion=bundle(rclcpp::Time{}.seconds(),false,0,.82);
  rclcpp::offline_publish("/rc_ctrl/button",excursion.joy);
  std::this_thread::sleep_for(std::chrono::milliseconds(4));
  const auto returned=bundle(rclcpp::Time{}.seconds(),false,0,.799);
  rclcpp::offline_publish("/rc_ctrl/button",returned.joy);
  // Neither new Joy has a matching Pose. A return to .799 cannot restore
  // authorization of the older .799 request after the .021 excursion.
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  const auto old_positions=count("tio_position");
  std::cout<<"REQUEST_LATCH deadband=0.02 held=.81->.799 request=.799 Joy=.82->.799 old_position_writes="
    <<old_positions<<'\n';
  for(const auto &e:snapshot()) if(e.name=="tio_position")
    std::cout<<"REQUEST_LATCH old register=0x103 value="<<e.increment.tran.y<<'\n';
  if(old_positions) {
    node.reset();assert_owner();
    throw std::runtime_error("request-relative Trigger excursion did not latch cancellation of old 0x0103=201");
  }
  // Delivering the new .799 Pose now completes a distinct request stamp.
  // The cancelled request cannot be reused, but this new pairing may execute.
  const auto settings_before=count("tio_config");
  tracking(true,rclcpp::Time{}.seconds()); // independent healthy heartbeat, not a new frame/epoch
  rclcpp::offline_publish("/rc_ctrl/left_target",returned.pose);
  assert(wait([&]{return count("tio_config")>settings_before;}));
  // Replay the old .82 Joy while the replacement's speed write blocks.
  // It must not cancel this newer .799 request or renew any input deadline.
  rclcpp::offline_publish("/rc_ctrl/button",excursion.joy);
  const bool completed=wait([]{return count("tio_position")==1;});
  bool new_target=false;
  for(const auto &e:snapshot()) if(e.name=="tio_position" && e.increment.tran.y==201) new_target=true;
  node.reset();assert_owner();
  if(!completed) throw std::runtime_error("replacement .799 request did not finish after old Joy replay");
  if(!new_target) throw std::runtime_error("new complete .799 request did not execute 0x0103=201");
  std::cout<<"PASS request-relative .799 cancellation stays latched through .82/.799; new complete frame executes201; old Joy replay ignored\n";
}
void regression_trigger_cancel() {
  using namespace fake;
  auto node=start(.25);pump(24,false);
  set([]{tio_delay_ms=35;});
  const double old_source=rclcpp::Time{}.seconds();
  deliver(bundle(old_source,false,0,.8));
  assert(wait([]{return count("tio_config")>=1;}));
  const auto withdrawal=bundle(rclcpp::Time{}.seconds(),false,0,0);
  rclcpp::offline_publish("/rc_ctrl/button",withdrawal.joy);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  const auto stale_positions=count("tio_position");
  for(const auto &e:snapshot()) if(e.name=="tio_position")
    std::cout<<"CANCEL old register=0x103 value="<<e.increment.tran.y<<'\n';
  // The delayed matching Pose now authorizes the new request, not the old one.
  rclcpp::offline_publish("/rc_ctrl/left_target",withdrawal.pose);
  set([]{tio_delay_ms=0;});
  pump(48,false,0,0);
  const auto before_new=count("tio_position");
  set([]{tio_delay_ms=35;});
  const auto config_before=count("tio_config");
  deliver(bundle(rclcpp::Time{}.seconds(),false,0,.7));
  // On the fixed path, speed settings may still be pending after cancellation.
  assert(wait([&]{return count("tio_config")>config_before || count("tio_position")>before_new;}));
  // An old Joy replay has Trigger=0; it must not revoke the newer .7 request.
  rclcpp::offline_publish("/rc_ctrl/button",withdrawal.joy);
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  bool new_target=false;
  for(const auto &e:snapshot()) if(e.name=="tio_position" && e.increment.tran.y==300) new_target=true;
  node.reset();assert_owner();
  if(stale_positions) throw std::runtime_error("new valid unpaired Joy failed to cancel old Trigger position write");
  if(!new_target) throw std::runtime_error("old Joy replay cancelled newer complete Trigger request");
  std::cout<<"PASS newer unpaired Joy cancels old Trigger before position; new complete request succeeds; old replay ignored\n";
  regression_trigger_request_latch();
}
int main(int argc,char **argv) {
  if(argc>1) {
    try {
      if(std::string(argv[1])=="source") regression_source_deadline();
      else if(std::string(argv[1])=="pairing") regression_interleaved_pairing();
      else if(std::string(argv[1])=="validity") regression_invalid_reads_and_restart();
      else if(std::string(argv[1])=="inflight_epoch") regression_inflight_epoch();
      else if(std::string(argv[1])=="trigger_cancel") regression_trigger_cancel();
      else regression_gripper_deadline();
      return 0;
    } catch(const std::exception &e) { std::cerr<<"FAIL "<<e.what()<<'\n';return 1; }
  }
  using namespace fake;
  {
    auto node=start();pump(32,false);pump(64,true);
    for(const auto &e:snapshot()) if(e.name=="servo") assert(zero(e.increment));
    pump(80,true,200);assert(count("servo")>=12);
    const auto before=count("tio_position");pump(32,true,200,.8);assert(count("tio_position")==before);
    pump(48,false,200,.8);assert(count("tio_position")==before+1);
    auto copy=snapshot();bool ordered=false;
    bool disabled=false,wrote=false;
    for(const auto &e:copy) {
      if(e.name=="servo_disable") disabled=true;
      if(disabled && e.name=="tio_position") wrote=true;
      if(disabled && wrote && e.name=="servo_enable") ordered=true;
    }
    assert(ordered);
    const auto after=snapshot().size();pump(48,true,200,.8);
    copy=snapshot();for(std::size_t i=after;i<copy.size();++i) if(copy[i].name=="servo") assert(zero(copy[i].increment));
    pump(40,true,400,.8);const auto release=snapshot().size();pump(48,false,400,.8);
    copy=snapshot();
    // A release racing an already-started SDK call may leave that one in flight.
    bool first=true;for(std::size_t i=release;i<copy.size();++i) if(copy[i].name=="servo") {
      if(!first) assert(zero(copy[i].increment));
      first=false;
    }
    node.reset();assert_owner();assert(count("logout")==1);
    std::cout<<"PASS actual node owner, static hold, residual release and serialized deferred gripper\n";
  }
  {
    auto node=start();pump(24,false);set([]{tio_delay_ms=80;});pump(320,false,0,1);
    assert(wait([]{return count("logout")==1;}));
    assert(count("servo_enable")==1);assert_no_grip_recovery();node.reset();assert_owner();
    std::cout<<"PASS actual node long gripper fault, no servo resume or Grip recovery\n";
  }
  for(bool protective:{false,true}) {
    auto node=start();pump(24,false);
    set([&]{if(protective) protective_error=0x128000;else servo_return=-12;});
    assert(wait([]{return count("logout")==1;}));assert_no_grip_recovery();node.reset();assert_owner();
    std::cout<<"PASS actual node "<<(protective ? "controller protection" : "servo failure")<<" latches across Grip\n";
  }
  {
    auto node=start();set([]{servo_delay_ms=160;});
    assert(wait([]{return count("logout")==1;}));assert_no_grip_recovery();node.reset();assert_owner();
    std::cout<<"PASS actual node delayed SDK call faults without catch-up burst\n";
  }
  {
    auto node=start();pump(24,false);pump(40,true,0);pump(40,true,200);
    const double frozen=rclcpp::Time{}.seconds();
    const double end=steady_seconds()+.28;
    while(steady_seconds()<end) {
      input(true,200,0,true,frozen); // re-publishing a cached frame is not fresh input
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
    const auto after=snapshot().size();pump(48,true,400);
    auto copy=snapshot();
    for(std::size_t i=after;i<copy.size();++i) if(copy[i].name=="servo") assert(zero(copy[i].increment));
    pump(24,false,400);const auto reengage=snapshot().size();pump(48,true,400);
    copy=snapshot();
    for(std::size_t i=reengage;i<copy.size();++i) if(copy[i].name=="servo") assert(zero(copy[i].increment));
    node.reset();assert_owner();
    std::cout<<"PASS actual node cached source timeout discards residual and requires release\n";
  }
  {
    auto node=start();pump(24,false);pump(24,true);
    const auto startup_tio=count("tio_position");
    pump(48,true,200,.8);assert(count("tio_position")==startup_tio);
    // The bridge emits neutral Grip on source/tracking loss while retaining
    // Trigger. Pending gripper work must not run because of that synthetic release.
    pump_without_pose(24,false,false,.8);
    assert(count("tio_position")==startup_tio);
    const auto after_loss=snapshot().size();
    // Delayed pose topic: a true tracking heartbeat plus fresh Joy cannot make
    // a valid reference out of the old pose, even within its nominal timeout.
    pump_without_pose(32,true,true,.8);
    assert(count("tio_position")==startup_tio);
    pump(48,true,500,.8);
    auto copy=snapshot();
    for(std::size_t i=after_loss;i<copy.size();++i)
      if(copy[i].name=="servo") assert(zero(copy[i].increment));
    assert(count("tio_position")==startup_tio);
    // Only a fresh tracked operator release allows the deferred Trigger and a
    // subsequent fresh press establishes the new reference at x=500 mm.
    pump(48,false,500,.8);assert(count("tio_position")==startup_tio+1);
    pump(24,true,500,.8);
    const auto before_new_motion=snapshot().size();pump(64,true,510,.8);
    copy=snapshot();bool moved=false;
    for(std::size_t i=before_new_motion;i<copy.size();++i)
      if(copy[i].name=="servo" && !zero(copy[i].increment)) moved=true;
    assert(moved);
    node.reset();assert_owner();
    std::cout<<"PASS actual node unordered reacquisition cannot revive old pose or execute pending Trigger\n";
  }
  {
    auto node=start();pump(24,false);pump(24,true);
    const auto startup_tio=count("tio_position");
    pump(48,true,200,.8);assert(count("tio_position")==startup_tio);
    pump_without_pose(24,false,false,.8);
    const auto after_loss=snapshot().size();
    // Recovery Pose before Joy leaves a recent synthetic Grip=0 in the
    // mailbox. It is not a source-matched operator release and must not run TIO.
    pump_without_joy(32,500);
    assert(count("tio_position")==startup_tio);
    // Ordinary, fully ordered new frames with the physical Grip still held
    // also cannot acknowledge the synthetic neutral release during loss.
    pump(48,true,500,.8);
    auto copy=snapshot();
    for(std::size_t i=after_loss;i<copy.size();++i)
      if(copy[i].name=="servo") assert(zero(copy[i].increment));
    assert(count("tio_position")==startup_tio);
    // A valid paired release authorizes maintenance, and only the later press
    // starts a new controller/feedback reference.
    pump(48,false,500,.8);assert(count("tio_position")==startup_tio+1);
    const auto reengage=snapshot().size();pump(24,true,500,.8);
    copy=snapshot();
    for(std::size_t i=reengage;i<copy.size();++i)
      if(copy[i].name=="servo") assert(zero(copy[i].increment));
    const auto before_move=snapshot().size();pump(64,true,510,.8);
    copy=snapshot();bool moved=false;
    for(std::size_t i=before_move;i<copy.size();++i)
      if(copy[i].name=="servo" && !zero(copy[i].increment)) moved=true;
    assert(moved);
    node.reset();assert_owner();
    std::cout<<"PASS actual node ordered recovery needs a matched operator release; Pose-before-Joy cannot run pending Trigger\n";
  }
  {
    reset();set([]{status_return=-3;});auto node=std::make_shared<RcControlNode>();
    assert(wait([]{return count("logout")==1;}));node.reset();
    assert(count("servo_enable")==0 && count("tio_position")==0);assert_owner();
    std::cout<<"PASS actual node partial startup logs out on the same owner\n";
  }
}
