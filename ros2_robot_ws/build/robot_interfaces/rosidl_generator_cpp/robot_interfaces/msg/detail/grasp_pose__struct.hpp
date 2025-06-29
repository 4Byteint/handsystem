// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/GraspPose.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__GRASP_POSE__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__GRASP_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__GraspPose __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__GraspPose __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GraspPose_
{
  using Type = GraspPose_<ContainerAllocator>;

  explicit GraspPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 16>::iterator, float>(this->data.begin(), this->data.end(), 0.0f);
    }
  }

  explicit GraspPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 16>::iterator, float>(this->data.begin(), this->data.end(), 0.0f);
    }
  }

  // field types and members
  using _data_type =
    std::array<float, 16>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::array<float, 16> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::GraspPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::GraspPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::GraspPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::GraspPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::GraspPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::GraspPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::GraspPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::GraspPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::GraspPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::GraspPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__GraspPose
    std::shared_ptr<robot_interfaces::msg::GraspPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__GraspPose
    std::shared_ptr<robot_interfaces::msg::GraspPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraspPose_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraspPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraspPose_

// alias to use template instance with default allocator
using GraspPose =
  robot_interfaces::msg::GraspPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__GRASP_POSE__STRUCT_HPP_
