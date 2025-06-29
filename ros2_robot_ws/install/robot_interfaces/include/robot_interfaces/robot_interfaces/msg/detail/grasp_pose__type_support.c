// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_interfaces:msg/GraspPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_interfaces/msg/detail/grasp_pose__rosidl_typesupport_introspection_c.h"
#include "robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_interfaces/msg/detail/grasp_pose__functions.h"
#include "robot_interfaces/msg/detail/grasp_pose__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_interfaces__msg__GraspPose__init(message_memory);
}

void robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_fini_function(void * message_memory)
{
  robot_interfaces__msg__GraspPose__fini(message_memory);
}

size_t robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__size_function__GraspPose__data(
  const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__get_const_function__GraspPose__data(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__get_function__GraspPose__data(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__fetch_function__GraspPose__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__get_const_function__GraspPose__data(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__assign_function__GraspPose__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__get_function__GraspPose__data(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(robot_interfaces__msg__GraspPose, data),  // bytes offset in struct
    NULL,  // default value
    robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__size_function__GraspPose__data,  // size() function pointer
    robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__get_const_function__GraspPose__data,  // get_const(index) function pointer
    robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__get_function__GraspPose__data,  // get(index) function pointer
    robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__fetch_function__GraspPose__data,  // fetch(index, &value) function pointer
    robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__assign_function__GraspPose__data,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_message_members = {
  "robot_interfaces__msg",  // message namespace
  "GraspPose",  // message name
  1,  // number of fields
  sizeof(robot_interfaces__msg__GraspPose),
  robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_message_member_array,  // message members
  robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_message_type_support_handle = {
  0,
  &robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, msg, GraspPose)() {
  if (!robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_message_type_support_handle.typesupport_identifier) {
    robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_interfaces__msg__GraspPose__rosidl_typesupport_introspection_c__GraspPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
