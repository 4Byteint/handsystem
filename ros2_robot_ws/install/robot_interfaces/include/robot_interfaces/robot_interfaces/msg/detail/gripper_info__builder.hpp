// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/GripperInfo.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__GRIPPER_INFO__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__GRIPPER_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/gripper_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_GripperInfo_result
{
public:
  Init_GripperInfo_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_interfaces::msg::GripperInfo result(::robot_interfaces::msg::GripperInfo::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::GripperInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::GripperInfo>()
{
  return robot_interfaces::msg::builder::Init_GripperInfo_result();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__GRIPPER_INFO__BUILDER_HPP_
