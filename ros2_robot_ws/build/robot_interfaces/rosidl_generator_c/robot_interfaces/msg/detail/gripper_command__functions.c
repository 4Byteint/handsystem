// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_interfaces:msg/GripperCommand.idl
// generated code does not contain a copyright notice
#include "robot_interfaces/msg/detail/gripper_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_interfaces__msg__GripperCommand__init(robot_interfaces__msg__GripperCommand * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // num
  // resp
  return true;
}

void
robot_interfaces__msg__GripperCommand__fini(robot_interfaces__msg__GripperCommand * msg)
{
  if (!msg) {
    return;
  }
  // id
  // num
  // resp
}

bool
robot_interfaces__msg__GripperCommand__are_equal(const robot_interfaces__msg__GripperCommand * lhs, const robot_interfaces__msg__GripperCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // num
  if (lhs->num != rhs->num) {
    return false;
  }
  // resp
  if (lhs->resp != rhs->resp) {
    return false;
  }
  return true;
}

bool
robot_interfaces__msg__GripperCommand__copy(
  const robot_interfaces__msg__GripperCommand * input,
  robot_interfaces__msg__GripperCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // num
  output->num = input->num;
  // resp
  output->resp = input->resp;
  return true;
}

robot_interfaces__msg__GripperCommand *
robot_interfaces__msg__GripperCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__GripperCommand * msg = (robot_interfaces__msg__GripperCommand *)allocator.allocate(sizeof(robot_interfaces__msg__GripperCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_interfaces__msg__GripperCommand));
  bool success = robot_interfaces__msg__GripperCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_interfaces__msg__GripperCommand__destroy(robot_interfaces__msg__GripperCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_interfaces__msg__GripperCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_interfaces__msg__GripperCommand__Sequence__init(robot_interfaces__msg__GripperCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__GripperCommand * data = NULL;

  if (size) {
    data = (robot_interfaces__msg__GripperCommand *)allocator.zero_allocate(size, sizeof(robot_interfaces__msg__GripperCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_interfaces__msg__GripperCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_interfaces__msg__GripperCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_interfaces__msg__GripperCommand__Sequence__fini(robot_interfaces__msg__GripperCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_interfaces__msg__GripperCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_interfaces__msg__GripperCommand__Sequence *
robot_interfaces__msg__GripperCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__GripperCommand__Sequence * array = (robot_interfaces__msg__GripperCommand__Sequence *)allocator.allocate(sizeof(robot_interfaces__msg__GripperCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_interfaces__msg__GripperCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_interfaces__msg__GripperCommand__Sequence__destroy(robot_interfaces__msg__GripperCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_interfaces__msg__GripperCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_interfaces__msg__GripperCommand__Sequence__are_equal(const robot_interfaces__msg__GripperCommand__Sequence * lhs, const robot_interfaces__msg__GripperCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_interfaces__msg__GripperCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_interfaces__msg__GripperCommand__Sequence__copy(
  const robot_interfaces__msg__GripperCommand__Sequence * input,
  robot_interfaces__msg__GripperCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_interfaces__msg__GripperCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_interfaces__msg__GripperCommand * data =
      (robot_interfaces__msg__GripperCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_interfaces__msg__GripperCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_interfaces__msg__GripperCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_interfaces__msg__GripperCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
