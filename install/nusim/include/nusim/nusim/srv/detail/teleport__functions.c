// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nusim:srv/Teleport.idl
// generated code does not contain a copyright notice
#include "nusim/srv/detail/teleport__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
nusim__srv__Teleport_Request__init(nusim__srv__Teleport_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // theta
  return true;
}

void
nusim__srv__Teleport_Request__fini(nusim__srv__Teleport_Request * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // theta
}

bool
nusim__srv__Teleport_Request__are_equal(const nusim__srv__Teleport_Request * lhs, const nusim__srv__Teleport_Request * rhs)
{
  if (!lhs || !rhs) {
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
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  return true;
}

bool
nusim__srv__Teleport_Request__copy(
  const nusim__srv__Teleport_Request * input,
  nusim__srv__Teleport_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // theta
  output->theta = input->theta;
  return true;
}

nusim__srv__Teleport_Request *
nusim__srv__Teleport_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nusim__srv__Teleport_Request * msg = (nusim__srv__Teleport_Request *)allocator.allocate(sizeof(nusim__srv__Teleport_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nusim__srv__Teleport_Request));
  bool success = nusim__srv__Teleport_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nusim__srv__Teleport_Request__destroy(nusim__srv__Teleport_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nusim__srv__Teleport_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nusim__srv__Teleport_Request__Sequence__init(nusim__srv__Teleport_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nusim__srv__Teleport_Request * data = NULL;

  if (size) {
    data = (nusim__srv__Teleport_Request *)allocator.zero_allocate(size, sizeof(nusim__srv__Teleport_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nusim__srv__Teleport_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nusim__srv__Teleport_Request__fini(&data[i - 1]);
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
nusim__srv__Teleport_Request__Sequence__fini(nusim__srv__Teleport_Request__Sequence * array)
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
      nusim__srv__Teleport_Request__fini(&array->data[i]);
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

nusim__srv__Teleport_Request__Sequence *
nusim__srv__Teleport_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nusim__srv__Teleport_Request__Sequence * array = (nusim__srv__Teleport_Request__Sequence *)allocator.allocate(sizeof(nusim__srv__Teleport_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nusim__srv__Teleport_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nusim__srv__Teleport_Request__Sequence__destroy(nusim__srv__Teleport_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nusim__srv__Teleport_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nusim__srv__Teleport_Request__Sequence__are_equal(const nusim__srv__Teleport_Request__Sequence * lhs, const nusim__srv__Teleport_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nusim__srv__Teleport_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nusim__srv__Teleport_Request__Sequence__copy(
  const nusim__srv__Teleport_Request__Sequence * input,
  nusim__srv__Teleport_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nusim__srv__Teleport_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nusim__srv__Teleport_Request * data =
      (nusim__srv__Teleport_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nusim__srv__Teleport_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nusim__srv__Teleport_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nusim__srv__Teleport_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
nusim__srv__Teleport_Response__init(nusim__srv__Teleport_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
nusim__srv__Teleport_Response__fini(nusim__srv__Teleport_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
nusim__srv__Teleport_Response__are_equal(const nusim__srv__Teleport_Response * lhs, const nusim__srv__Teleport_Response * rhs)
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
nusim__srv__Teleport_Response__copy(
  const nusim__srv__Teleport_Response * input,
  nusim__srv__Teleport_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

nusim__srv__Teleport_Response *
nusim__srv__Teleport_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nusim__srv__Teleport_Response * msg = (nusim__srv__Teleport_Response *)allocator.allocate(sizeof(nusim__srv__Teleport_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nusim__srv__Teleport_Response));
  bool success = nusim__srv__Teleport_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nusim__srv__Teleport_Response__destroy(nusim__srv__Teleport_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nusim__srv__Teleport_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nusim__srv__Teleport_Response__Sequence__init(nusim__srv__Teleport_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nusim__srv__Teleport_Response * data = NULL;

  if (size) {
    data = (nusim__srv__Teleport_Response *)allocator.zero_allocate(size, sizeof(nusim__srv__Teleport_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nusim__srv__Teleport_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nusim__srv__Teleport_Response__fini(&data[i - 1]);
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
nusim__srv__Teleport_Response__Sequence__fini(nusim__srv__Teleport_Response__Sequence * array)
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
      nusim__srv__Teleport_Response__fini(&array->data[i]);
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

nusim__srv__Teleport_Response__Sequence *
nusim__srv__Teleport_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nusim__srv__Teleport_Response__Sequence * array = (nusim__srv__Teleport_Response__Sequence *)allocator.allocate(sizeof(nusim__srv__Teleport_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nusim__srv__Teleport_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nusim__srv__Teleport_Response__Sequence__destroy(nusim__srv__Teleport_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nusim__srv__Teleport_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nusim__srv__Teleport_Response__Sequence__are_equal(const nusim__srv__Teleport_Response__Sequence * lhs, const nusim__srv__Teleport_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nusim__srv__Teleport_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nusim__srv__Teleport_Response__Sequence__copy(
  const nusim__srv__Teleport_Response__Sequence * input,
  nusim__srv__Teleport_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nusim__srv__Teleport_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nusim__srv__Teleport_Response * data =
      (nusim__srv__Teleport_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nusim__srv__Teleport_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nusim__srv__Teleport_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nusim__srv__Teleport_Response__copy(
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
// #include "nusim/srv/detail/teleport__functions.h"

bool
nusim__srv__Teleport_Event__init(nusim__srv__Teleport_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    nusim__srv__Teleport_Event__fini(msg);
    return false;
  }
  // request
  if (!nusim__srv__Teleport_Request__Sequence__init(&msg->request, 0)) {
    nusim__srv__Teleport_Event__fini(msg);
    return false;
  }
  // response
  if (!nusim__srv__Teleport_Response__Sequence__init(&msg->response, 0)) {
    nusim__srv__Teleport_Event__fini(msg);
    return false;
  }
  return true;
}

void
nusim__srv__Teleport_Event__fini(nusim__srv__Teleport_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  nusim__srv__Teleport_Request__Sequence__fini(&msg->request);
  // response
  nusim__srv__Teleport_Response__Sequence__fini(&msg->response);
}

bool
nusim__srv__Teleport_Event__are_equal(const nusim__srv__Teleport_Event * lhs, const nusim__srv__Teleport_Event * rhs)
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
  if (!nusim__srv__Teleport_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!nusim__srv__Teleport_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
nusim__srv__Teleport_Event__copy(
  const nusim__srv__Teleport_Event * input,
  nusim__srv__Teleport_Event * output)
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
  if (!nusim__srv__Teleport_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!nusim__srv__Teleport_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

nusim__srv__Teleport_Event *
nusim__srv__Teleport_Event__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nusim__srv__Teleport_Event * msg = (nusim__srv__Teleport_Event *)allocator.allocate(sizeof(nusim__srv__Teleport_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nusim__srv__Teleport_Event));
  bool success = nusim__srv__Teleport_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nusim__srv__Teleport_Event__destroy(nusim__srv__Teleport_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nusim__srv__Teleport_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nusim__srv__Teleport_Event__Sequence__init(nusim__srv__Teleport_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nusim__srv__Teleport_Event * data = NULL;

  if (size) {
    data = (nusim__srv__Teleport_Event *)allocator.zero_allocate(size, sizeof(nusim__srv__Teleport_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nusim__srv__Teleport_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nusim__srv__Teleport_Event__fini(&data[i - 1]);
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
nusim__srv__Teleport_Event__Sequence__fini(nusim__srv__Teleport_Event__Sequence * array)
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
      nusim__srv__Teleport_Event__fini(&array->data[i]);
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

nusim__srv__Teleport_Event__Sequence *
nusim__srv__Teleport_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nusim__srv__Teleport_Event__Sequence * array = (nusim__srv__Teleport_Event__Sequence *)allocator.allocate(sizeof(nusim__srv__Teleport_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nusim__srv__Teleport_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nusim__srv__Teleport_Event__Sequence__destroy(nusim__srv__Teleport_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nusim__srv__Teleport_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nusim__srv__Teleport_Event__Sequence__are_equal(const nusim__srv__Teleport_Event__Sequence * lhs, const nusim__srv__Teleport_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nusim__srv__Teleport_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nusim__srv__Teleport_Event__Sequence__copy(
  const nusim__srv__Teleport_Event__Sequence * input,
  nusim__srv__Teleport_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nusim__srv__Teleport_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nusim__srv__Teleport_Event * data =
      (nusim__srv__Teleport_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nusim__srv__Teleport_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nusim__srv__Teleport_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nusim__srv__Teleport_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
