// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/GripperCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__GRIPPER_COMMAND__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__GRIPPER_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__GripperCommand __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__GripperCommand __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GripperCommand_
{
  using Type = GripperCommand_<ContainerAllocator>;

  explicit GripperCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->num = 0l;
      this->resp = 0l;
    }
  }

  explicit GripperCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->num = 0l;
      this->resp = 0l;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _num_type =
    int32_t;
  _num_type num;
  using _resp_type =
    int32_t;
  _resp_type resp;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__num(
    const int32_t & _arg)
  {
    this->num = _arg;
    return *this;
  }
  Type & set__resp(
    const int32_t & _arg)
  {
    this->resp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::GripperCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::GripperCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::GripperCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::GripperCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::GripperCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::GripperCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::GripperCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::GripperCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::GripperCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::GripperCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__GripperCommand
    std::shared_ptr<robot_interfaces::msg::GripperCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__GripperCommand
    std::shared_ptr<robot_interfaces::msg::GripperCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripperCommand_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->num != other.num) {
      return false;
    }
    if (this->resp != other.resp) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripperCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripperCommand_

// alias to use template instance with default allocator
using GripperCommand =
  robot_interfaces::msg::GripperCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__GRIPPER_COMMAND__STRUCT_HPP_
