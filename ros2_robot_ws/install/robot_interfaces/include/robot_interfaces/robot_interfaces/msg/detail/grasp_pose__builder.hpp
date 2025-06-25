// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/GraspPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__GRASP_POSE__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__GRASP_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/grasp_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_GraspPose_angle
{
public:
  explicit Init_GraspPose_angle(::robot_interfaces::msg::GraspPose & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::GraspPose angle(::robot_interfaces::msg::GraspPose::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::GraspPose msg_;
};

class Init_GraspPose_z
{
public:
  explicit Init_GraspPose_z(::robot_interfaces::msg::GraspPose & msg)
  : msg_(msg)
  {}
  Init_GraspPose_angle z(::robot_interfaces::msg::GraspPose::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_GraspPose_angle(msg_);
  }

private:
  ::robot_interfaces::msg::GraspPose msg_;
};

class Init_GraspPose_y
{
public:
  explicit Init_GraspPose_y(::robot_interfaces::msg::GraspPose & msg)
  : msg_(msg)
  {}
  Init_GraspPose_z y(::robot_interfaces::msg::GraspPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GraspPose_z(msg_);
  }

private:
  ::robot_interfaces::msg::GraspPose msg_;
};

class Init_GraspPose_x
{
public:
  Init_GraspPose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GraspPose_y x(::robot_interfaces::msg::GraspPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GraspPose_y(msg_);
  }

private:
  ::robot_interfaces::msg::GraspPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::GraspPose>()
{
  return robot_interfaces::msg::builder::Init_GraspPose_x();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__GRASP_POSE__BUILDER_HPP_
