#pragma once

// Declaration-only ROS substitute for the explicitly isolated offline syntax
// check. Never place this directory on a production target's include path.
#ifndef XRTOOLKIT_OFFLINE_ROS_SHIM
#error "ROS shims are only available to the offline declaration compatibility check"
#endif

#include <chrono>
#include <any>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace rclcpp {
inline std::unordered_map<std::string, std::any> offline_parameters;
template<class Message>
inline std::unordered_map<std::string,
  std::function<void(std::shared_ptr<Message>)>> offline_callbacks;
template<class Message>
void offline_publish(const std::string & topic, const Message & message) {
  offline_callbacks<Message>.at(topic)(std::make_shared<Message>(message));
}
struct Logger {};
inline Logger get_logger(const std::string &) { return {}; }
template<class... Args> inline void offline_log(Args &&...) {}
class Time {
 public:
  double seconds() const {
    return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }
  int64_t nanoseconds() const { return static_cast<int64_t>(seconds() * 1e9); }
};
class Clock {
 public:
  using SharedPtr = std::shared_ptr<Clock>;
  Time now() const { return {}; }
};
class QoS {
 public:
  explicit QoS(std::size_t) {}
};
template<class Message> struct Subscription {
  using SharedPtr = std::shared_ptr<Subscription<Message>>;
};
struct TimerBase { using SharedPtr = std::shared_ptr<TimerBase>; };
class Node {
 public:
  using SharedPtr = std::shared_ptr<Node>;
  explicit Node(const std::string &) {}
  static SharedPtr make_shared(const std::string & name) {
    return std::make_shared<Node>(name);
  }
  virtual ~Node() = default;
  template<class T> T declare_parameter(const std::string & name, const T & value) {
    auto it = offline_parameters.find(name);
    if (it != offline_parameters.end()) return std::any_cast<T>(it->second);
    return value;
  }
  template<class Message, class Callback>
  typename Subscription<Message>::SharedPtr create_subscription(
      const std::string & topic, const QoS &, Callback && callback) {
    offline_callbacks<Message>[topic] = std::forward<Callback>(callback);
    return std::make_shared<Subscription<Message>>();
  }
  template<class Duration, class Callback>
  TimerBase::SharedPtr create_wall_timer(Duration, Callback &&) {
    return std::make_shared<TimerBase>();
  }
  Logger get_logger() const { return {}; }
  Time now() const { return {}; }
  Clock::SharedPtr get_clock() const { return std::make_shared<Clock>(); }
};
inline void init(int, char **) {}
inline bool ok() { return true; }
template<class T> inline void spin(const std::shared_ptr<T> &) {}
inline void shutdown() {}
}  // namespace rclcpp

#define RCLCPP_INFO(...) ::rclcpp::offline_log(__VA_ARGS__)
#define RCLCPP_WARN(...) ::rclcpp::offline_log(__VA_ARGS__)
#define RCLCPP_ERROR(...) ::rclcpp::offline_log(__VA_ARGS__)
#define RCLCPP_FATAL(...) ::rclcpp::offline_log(__VA_ARGS__)
#define RCLCPP_DEBUG(...) ::rclcpp::offline_log(__VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(...) ::rclcpp::offline_log(__VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(...) ::rclcpp::offline_log(__VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(...) ::rclcpp::offline_log(__VA_ARGS__)
