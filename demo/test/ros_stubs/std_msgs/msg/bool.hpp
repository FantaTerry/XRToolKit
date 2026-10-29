#pragma once
#include <memory>
namespace std_msgs { namespace msg {
struct Bool {
  using SharedPtr = std::shared_ptr<Bool>;
  using ConstSharedPtr = std::shared_ptr<const Bool>;
  bool data = false;
};
}}  // namespace std_msgs::msg
