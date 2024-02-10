// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nuturtle_control:srv/Control.idl
// generated code does not contain a copyright notice
#include "nuturtle_control/srv/detail/control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
nuturtle_control__srv__Control_Request__init(nuturtle_control__srv__Control_Request * msg)
{
  if (!msg) {
    return false;
  }
  // velocity
  // radius
  return true;
}

void
nuturtle_control__srv__Control_Request__fini(nuturtle_control__srv__Control_Request * msg)
{
  if (!msg) {
    return;
  }
  // velocity
  // radius
}

bool
nuturtle_control__srv__Control_Request__are_equal(const nuturtle_control__srv__Control_Request * lhs, const nuturtle_control__srv__Control_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // radius
  if (lhs->radius != rhs->radius) {
    return false;
  }
  return true;
}

bool
nuturtle_control__srv__Control_Request__copy(
  const nuturtle_control__srv__Control_Request * input,
  nuturtle_control__srv__Control_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // velocity
  output->velocity = input->velocity;
  // radius
  output->radius = input->radius;
  return true;
}

nuturtle_control__srv__Control_Request *
nuturtle_control__srv__Control_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__srv__Control_Request * msg = (nuturtle_control__srv__Control_Request *)allocator.allocate(sizeof(nuturtle_control__srv__Control_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nuturtle_control__srv__Control_Request));
  bool success = nuturtle_control__srv__Control_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nuturtle_control__srv__Control_Request__destroy(nuturtle_control__srv__Control_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nuturtle_control__srv__Control_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nuturtle_control__srv__Control_Request__Sequence__init(nuturtle_control__srv__Control_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__srv__Control_Request * data = NULL;

  if (size) {
    data = (nuturtle_control__srv__Control_Request *)allocator.zero_allocate(size, sizeof(nuturtle_control__srv__Control_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nuturtle_control__srv__Control_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nuturtle_control__srv__Control_Request__fini(&data[i - 1]);
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
nuturtle_control__srv__Control_Request__Sequence__fini(nuturtle_control__srv__Control_Request__Sequence * array)
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
      nuturtle_control__srv__Control_Request__fini(&array->data[i]);
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

nuturtle_control__srv__Control_Request__Sequence *
nuturtle_control__srv__Control_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__srv__Control_Request__Sequence * array = (nuturtle_control__srv__Control_Request__Sequence *)allocator.allocate(sizeof(nuturtle_control__srv__Control_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nuturtle_control__srv__Control_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nuturtle_control__srv__Control_Request__Sequence__destroy(nuturtle_control__srv__Control_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nuturtle_control__srv__Control_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nuturtle_control__srv__Control_Request__Sequence__are_equal(const nuturtle_control__srv__Control_Request__Sequence * lhs, const nuturtle_control__srv__Control_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nuturtle_control__srv__Control_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nuturtle_control__srv__Control_Request__Sequence__copy(
  const nuturtle_control__srv__Control_Request__Sequence * input,
  nuturtle_control__srv__Control_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nuturtle_control__srv__Control_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nuturtle_control__srv__Control_Request * data =
      (nuturtle_control__srv__Control_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nuturtle_control__srv__Control_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nuturtle_control__srv__Control_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nuturtle_control__srv__Control_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
nuturtle_control__srv__Control_Response__init(nuturtle_control__srv__Control_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
nuturtle_control__srv__Control_Response__fini(nuturtle_control__srv__Control_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
nuturtle_control__srv__Control_Response__are_equal(const nuturtle_control__srv__Control_Response * lhs, const nuturtle_control__srv__Control_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
nuturtle_control__srv__Control_Response__copy(
  const nuturtle_control__srv__Control_Response * input,
  nuturtle_control__srv__Control_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

nuturtle_control__srv__Control_Response *
nuturtle_control__srv__Control_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__srv__Control_Response * msg = (nuturtle_control__srv__Control_Response *)allocator.allocate(sizeof(nuturtle_control__srv__Control_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nuturtle_control__srv__Control_Response));
  bool success = nuturtle_control__srv__Control_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nuturtle_control__srv__Control_Response__destroy(nuturtle_control__srv__Control_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nuturtle_control__srv__Control_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nuturtle_control__srv__Control_Response__Sequence__init(nuturtle_control__srv__Control_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__srv__Control_Response * data = NULL;

  if (size) {
    data = (nuturtle_control__srv__Control_Response *)allocator.zero_allocate(size, sizeof(nuturtle_control__srv__Control_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nuturtle_control__srv__Control_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nuturtle_control__srv__Control_Response__fini(&data[i - 1]);
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
nuturtle_control__srv__Control_Response__Sequence__fini(nuturtle_control__srv__Control_Response__Sequence * array)
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
      nuturtle_control__srv__Control_Response__fini(&array->data[i]);
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

nuturtle_control__srv__Control_Response__Sequence *
nuturtle_control__srv__Control_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__srv__Control_Response__Sequence * array = (nuturtle_control__srv__Control_Response__Sequence *)allocator.allocate(sizeof(nuturtle_control__srv__Control_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nuturtle_control__srv__Control_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nuturtle_control__srv__Control_Response__Sequence__destroy(nuturtle_control__srv__Control_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nuturtle_control__srv__Control_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nuturtle_control__srv__Control_Response__Sequence__are_equal(const nuturtle_control__srv__Control_Response__Sequence * lhs, const nuturtle_control__srv__Control_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nuturtle_control__srv__Control_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nuturtle_control__srv__Control_Response__Sequence__copy(
  const nuturtle_control__srv__Control_Response__Sequence * input,
  nuturtle_control__srv__Control_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nuturtle_control__srv__Control_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nuturtle_control__srv__Control_Response * data =
      (nuturtle_control__srv__Control_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nuturtle_control__srv__Control_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nuturtle_control__srv__Control_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nuturtle_control__srv__Control_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "nuturtle_control/srv/detail/control__functions.h"

bool
nuturtle_control__srv__Control_Event__init(nuturtle_control__srv__Control_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    nuturtle_control__srv__Control_Event__fini(msg);
    return false;
  }
  // request
  if (!nuturtle_control__srv__Control_Request__Sequence__init(&msg->request, 0)) {
    nuturtle_control__srv__Control_Event__fini(msg);
    return false;
  }
  // response
  if (!nuturtle_control__srv__Control_Response__Sequence__init(&msg->response, 0)) {
    nuturtle_control__srv__Control_Event__fini(msg);
    return false;
  }
  return true;
}

void
nuturtle_control__srv__Control_Event__fini(nuturtle_control__srv__Control_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  nuturtle_control__srv__Control_Request__Sequence__fini(&msg->request);
  // response
  nuturtle_control__srv__Control_Response__Sequence__fini(&msg->response);
}

bool
nuturtle_control__srv__Control_Event__are_equal(const nuturtle_control__srv__Control_Event * lhs, const nuturtle_control__srv__Control_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!nuturtle_control__srv__Control_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!nuturtle_control__srv__Control_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
nuturtle_control__srv__Control_Event__copy(
  const nuturtle_control__srv__Control_Event * input,
  nuturtle_control__srv__Control_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!nuturtle_control__srv__Control_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!nuturtle_control__srv__Control_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

nuturtle_control__srv__Control_Event *
nuturtle_control__srv__Control_Event__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__srv__Control_Event * msg = (nuturtle_control__srv__Control_Event *)allocator.allocate(sizeof(nuturtle_control__srv__Control_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nuturtle_control__srv__Control_Event));
  bool success = nuturtle_control__srv__Control_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nuturtle_control__srv__Control_Event__destroy(nuturtle_control__srv__Control_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nuturtle_control__srv__Control_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nuturtle_control__srv__Control_Event__Sequence__init(nuturtle_control__srv__Control_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__srv__Control_Event * data = NULL;

  if (size) {
    data = (nuturtle_control__srv__Control_Event *)allocator.zero_allocate(size, sizeof(nuturtle_control__srv__Control_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nuturtle_control__srv__Control_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nuturtle_control__srv__Control_Event__fini(&data[i - 1]);
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
nuturtle_control__srv__Control_Event__Sequence__fini(nuturtle_control__srv__Control_Event__Sequence * array)
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
      nuturtle_control__srv__Control_Event__fini(&array->data[i]);
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

nuturtle_control__srv__Control_Event__Sequence *
nuturtle_control__srv__Control_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuturtle_control__srv__Control_Event__Sequence * array = (nuturtle_control__srv__Control_Event__Sequence *)allocator.allocate(sizeof(nuturtle_control__srv__Control_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nuturtle_control__srv__Control_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nuturtle_control__srv__Control_Event__Sequence__destroy(nuturtle_control__srv__Control_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nuturtle_control__srv__Control_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nuturtle_control__srv__Control_Event__Sequence__are_equal(const nuturtle_control__srv__Control_Event__Sequence * lhs, const nuturtle_control__srv__Control_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nuturtle_control__srv__Control_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nuturtle_control__srv__Control_Event__Sequence__copy(
  const nuturtle_control__srv__Control_Event__Sequence * input,
  nuturtle_control__srv__Control_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nuturtle_control__srv__Control_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nuturtle_control__srv__Control_Event * data =
      (nuturtle_control__srv__Control_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nuturtle_control__srv__Control_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nuturtle_control__srv__Control_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nuturtle_control__srv__Control_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
