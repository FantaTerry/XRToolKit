// Executes the production owner loop with an in-memory ROS mailbox and a fake
// implementation of the bundled SDK declarations. No vendor library is linked.
#include <cassert>
#include <cstring>
#include <functional>
#include <iostream>
#include <vector>
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
  j.header=p.header; j.axes={grip ? 1.f : 0.f,0.f,static_cast<float>(trigger),0.f};
  std_msgs::msg::Bool t; t.data=tracked;
  rclcpp::offline_publish("/rc_ctrl/left_tracking",t);
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
    j.axes={grip ? 1.f : 0.f,0.f,static_cast<float>(trigger),0.f};
    std_msgs::msg::Bool t;t.data=tracked;
    rclcpp::offline_publish("/rc_ctrl/left_tracking",t);
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
    std_msgs::msg::Bool t;t.data=true;
    rclcpp::offline_publish("/rc_ctrl/left_tracking",t);
    rclcpp::offline_publish("/rc_ctrl/left_target",p);
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}
std::shared_ptr<RcControlNode> start() {
  reset(); rclcpp::offline_parameters.clear();
  rclcpp::offline_parameters["gripper_initialize"]=false;
  rclcpp::offline_parameters["max_servo_interval_s"]=.12;
  rclcpp::offline_parameters["feedback_period_s"]=.03;
  rclcpp::offline_parameters["gripper_call_timeout_s"]=.03;
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
errno_t JAKAZuRobot::set_status_data_update_time_interval(float) {fake::add("status_interval");return 0;}
errno_t JAKAZuRobot::set_block_wait_timeout(float) {fake::add("block_timeout");return 0;}
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
errno_t JAKAZuRobot::send_tio_rs_command(int,uint8_t *,int) {
  fake::add("tio");int delay;
  {std::lock_guard<std::mutex> l(fake::mutex);assert(!fake::servo_enabled);delay=fake::tio_delay_ms;}
  if(delay) std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  return 0;
}

int main() {
  using namespace fake;
  {
    auto node=start();pump(32,false);pump(64,true);
    for(const auto &e:snapshot()) if(e.name=="servo") assert(zero(e.increment));
    pump(80,true,200);assert(count("servo")>=12);
    const auto before=count("tio");pump(32,true,200,.8);assert(count("tio")==before);
    pump(48,false,200,.8);assert(count("tio")==before+1);
    auto copy=snapshot();bool ordered=false;
    for(std::size_t i=0;i+2<copy.size();++i)
      if(copy[i].name=="servo_disable" && copy[i+1].name=="tio" && copy[i+2].name=="servo_enable") ordered=true;
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
    auto node=start();set([]{tio_delay_ms=80;});pump(120,false,0,1);
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
    const auto startup_tio=count("tio");
    pump(48,true,200,.8);assert(count("tio")==startup_tio);
    // The bridge emits neutral Grip on source/tracking loss while retaining
    // Trigger. Pending gripper work must not run because of that synthetic release.
    pump_without_pose(24,false,false,.8);
    assert(count("tio")==startup_tio);
    const auto after_loss=snapshot().size();
    // Delayed pose topic: a true tracking heartbeat plus fresh Joy cannot make
    // a valid reference out of the old pose, even within its nominal timeout.
    pump_without_pose(32,true,true,.8);
    assert(count("tio")==startup_tio);
    pump(48,true,500,.8);
    auto copy=snapshot();
    for(std::size_t i=after_loss;i<copy.size();++i)
      if(copy[i].name=="servo") assert(zero(copy[i].increment));
    assert(count("tio")==startup_tio);
    // Only a fresh tracked operator release allows the deferred Trigger and a
    // subsequent fresh press establishes the new reference at x=500 mm.
    pump(48,false,500,.8);assert(count("tio")==startup_tio+1);
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
    const auto startup_tio=count("tio");
    pump(48,true,200,.8);assert(count("tio")==startup_tio);
    pump_without_pose(24,false,false,.8);
    const auto after_loss=snapshot().size();
    // Recovery Pose before Joy leaves a recent synthetic Grip=0 in the
    // mailbox. It is not a source-matched operator release and must not run TIO.
    pump_without_joy(32,500);
    assert(count("tio")==startup_tio);
    // Ordinary, fully ordered new frames with the physical Grip still held
    // also cannot acknowledge the synthetic neutral release during loss.
    pump(48,true,500,.8);
    auto copy=snapshot();
    for(std::size_t i=after_loss;i<copy.size();++i)
      if(copy[i].name=="servo") assert(zero(copy[i].increment));
    assert(count("tio")==startup_tio);
    // A valid paired release authorizes maintenance, and only the later press
    // starts a new controller/feedback reference.
    pump(48,false,500,.8);assert(count("tio")==startup_tio+1);
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
    assert(count("servo_enable")==0 && count("tio")==0);assert_owner();
    std::cout<<"PASS actual node partial startup logs out on the same owner\n";
  }
}
