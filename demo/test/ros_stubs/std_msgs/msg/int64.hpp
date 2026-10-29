#pragma once
#ifndef XRTOOLKIT_OFFLINE_ROS_SHIM
#error "ROS shims are only available to explicitly disconnected tests"
#endif
#include <cstdint>
#include <memory>
namespace std_msgs { namespace msg {
struct Int64 {
  using SharedPtr=std::shared_ptr<Int64>;
  int64_t data=0;
};
}}
