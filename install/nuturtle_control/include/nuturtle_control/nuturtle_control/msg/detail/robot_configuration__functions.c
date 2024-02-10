// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nuturtle_control:msg/RobotConfiguration.idl
// generated code does not contain a copyright notice
#include "nuturtle_control/msg/detail/robot_configuration__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
nuturtle_control__msg__RobotConfiguration__init(nuturtle_control__msg__RobotConfiguration * msg)
{
  if (!msg) {
    return false;
  }
  // theta
  // x
  // y
  return true;
}

void
nuturtle_control__msg__RobotConfiguration__fini(nuturtle_control__msg__RobotConfiguration * msg)
{
  if (!msg) {
    return;
  }
  // theta
  // x
  // y
}

bool
nuturtle_control__msg__RobotConfiguration__are_equal(const nuturtle_control__msg__RobotConfiguration * lhs, const nuturtle_control__msg__RobotConfiguration * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
nuturtle_control__msg__RobotConfiguration__copy(
  const nuturtle_control__msg__RobotConfiguration * input,
  nuturtle_control__msg__RobotConfiguration * output)
{
  if (!input || !output) {
    return false;
  }
  // theta
  output->theta = input->theta;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

nuturtle_control__msg__RobotConfiguration *
nuturtle_control__msg__RobotConfiguration__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__msg__RobotConfiguration * msg = (nuturtle_control__msg__RobotConfiguration *)allocator.allocate(sizeof(nuturtle_control__msg__RobotConfiguration), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nuturtle_control__msg__RobotConfiguration));
  bool success = nuturtle_control__msg__RobotConfiguration__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nuturtle_control__msg__RobotConfiguration__destroy(nuturtle_control__msg__RobotConfiguration * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nuturtle_control__msg__RobotConfiguration__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nuturtle_control__msg__RobotConfiguration__Sequence__init(nuturtle_control__msg__RobotConfiguration__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__msg__RobotConfiguration * data = NULL;

  if (size) {
    data = (nuturtle_control__msg__RobotConfiguration *)allocator.zero_allocate(size, sizeof(nuturtle_control__msg__RobotConfiguration), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nuturtle_control__msg__RobotConfiguration__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nuturtle_control__msg__RobotConfiguration__fini(&data[i - 1]);
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
nuturtle_control__msg__RobotConfiguration__Sequence__fini(nuturtle_control__msg__RobotConfiguration__Sequence * array)
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
      nuturtle_control__msg__RobotConfiguration__fini(&array->data[i]);
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

nuturtle_control__msg__RobotConfiguration__Sequence *
nuturtle_control__msg__RobotConfiguration__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__msg__RobotConfiguration__Sequence * array = (nuturtle_control__msg__RobotConfiguration__Sequence *)allocator.allocate(sizeof(nuturtle_control__msg__RobotConfiguration__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nuturtle_control__msg__RobotConfiguration__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nuturtle_control__msg__RobotConfiguration__Sequence__destroy(nuturtle_control__msg__RobotConfiguration__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nuturtle_control__msg__RobotConfiguration__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nuturtle_control__msg__RobotConfiguration__Sequence__are_equal(const nuturtle_control__msg__RobotConfiguration__Sequence * lhs, const nuturtle_control__msg__RobotConfiguration__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nuturtle_control__msg__RobotConfiguration__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nuturtle_control__msg__RobotConfiguration__Sequence__copy(
  const nuturtle_control__msg__RobotConfiguration__Sequence * input,
  nuturtle_control__msg__RobotConfiguration__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nuturtle_control__msg__RobotConfiguration);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nuturtle_control__msg__RobotConfiguration * data =
      (nuturtle_control__msg__RobotConfiguration *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nuturtle_control__msg__RobotConfiguration__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nuturtle_control__msg__RobotConfiguration__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nuturtle_control__msg__RobotConfiguration__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
