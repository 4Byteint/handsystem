// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/GripperCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__GRIPPER_COMMAND__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__GRIPPER_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/gripper_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_GripperCommand_resp
{
public:
  explicit Init_GripperCommand_resp(::robot_interfaces::msg::GripperCommand & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::GripperCommand resp(::robot_interfaces::msg::GripperCommand::_resp_type arg)
  {
    msg_.resp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::GripperCommand msg_;
};

class Init_GripperCommand_num
{
public:
  explicit Init_GripperCommand_num(::robot_interfaces::msg::GripperCommand & msg)
  : msg_(msg)
  {}
  Init_GripperCommand_resp num(::robot_interfaces::msg::GripperCommand::_num_type arg)
  {
    msg_.num = std::move(arg);
    return Init_GripperCommand_resp(msg_);
  }

private:
  ::robot_interfaces::msg::GripperCommand msg_;
};

class Init_GripperCommand_id
{
public:
  Init_GripperCommand_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperCommand_num id(::robot_interfaces::msg::GripperCommand::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_GripperCommand_num(msg_);
  }

private:
  ::robot_interfaces::msg::GripperCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::GripperCommand>()
{
  return robot_interfaces::msg::builder::Init_GripperCommand_id();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__GRIPPER_COMMAND__BUILDER_HPP_
