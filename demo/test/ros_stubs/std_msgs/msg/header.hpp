#pragma once
#include <cstdint>
#include <string>
namespace std_msgs { namespace msg {
struct Header {
  struct Stamp { int32_t sec = 0; uint32_t nanosec = 0; } stamp;
  std::string frame_id;
};
}}  // namespace std_msgs::msg
