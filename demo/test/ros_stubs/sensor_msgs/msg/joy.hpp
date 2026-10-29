#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#include "std_msgs/msg/header.hpp"
namespace sensor_msgs { namespace msg {
struct Joy {
  using SharedPtr = std::shared_ptr<Joy>;
  using ConstSharedPtr = std::shared_ptr<const Joy>;
  std_msgs::msg::Header header;
  std::vector<float> axes;
  std::vector<int32_t> buttons;
};
}}  // namespace sensor_msgs::msg
