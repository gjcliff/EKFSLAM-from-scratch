// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nuturtle_control:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef NUTURTLE_CONTROL__SRV__DETAIL__CONTROL__STRUCT_H_
#define NUTURTLE_CONTROL__SRV__DETAIL__CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Control in the package nuturtle_control.
typedef struct nuturtle_control__srv__Control_Request
{
  double velocity;
  /// the radius of the arc
  double radius;
} nuturtle_control__srv__Control_Request;

// Struct for a sequence of nuturtle_control__srv__Control_Request.
typedef struct nuturtle_control__srv__Control_Request__Sequence
{
  nuturtle_control__srv__Control_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nuturtle_control__srv__Control_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Control in the package nuturtle_control.
typedef struct nuturtle_control__srv__Control_Response
{
  uint8_t structure_needs_at_least_one_member;
} nuturtle_control__srv__Control_Response;

// Struct for a sequence of nuturtle_control__srv__Control_Response.
typedef struct nuturtle_control__srv__Control_Response__Sequence
{
  nuturtle_control__srv__Control_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nuturtle_control__srv__Control_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  nuturtle_control__srv__Control_Event__request__MAX_SIZE = 1
};
// response
enum
{
  nuturtle_control__srv__Control_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Control in the package nuturtle_control.
typedef struct nuturtle_control__srv__Control_Event
{
  service_msgs__msg__ServiceEventInfo info;
  nuturtle_control__srv__Control_Request__Sequence request;
  nuturtle_control__srv__Control_Response__Sequence response;
} nuturtle_control__srv__Control_Event;

// Struct for a sequence of nuturtle_control__srv__Control_Event.
typedef struct nuturtle_control__srv__Control_Event__Sequence
{
  nuturtle_control__srv__Control_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nuturtle_control__srv__Control_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NUTURTLE_CONTROL__SRV__DETAIL__CONTROL__STRUCT_H_
