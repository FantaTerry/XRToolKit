#pragma once
#include <memory>
#include "std_msgs/msg/header.hpp"
namespace geometry_msgs { namespace msg {
struct PoseStamped {
  using SharedPtr = std::shared_ptr<PoseStamped>;
  using ConstSharedPtr = std::shared_ptr<const PoseStamped>;
  std_msgs::msg::Header header;
  struct Pose {
    struct Point { double x = 0, y = 0, z = 0; } position;
    struct Quaternion { double x = 0, y = 0, z = 0, w = 0; } orientation;
  } pose;
};
}}  // namespace geometry_msgs::msg
