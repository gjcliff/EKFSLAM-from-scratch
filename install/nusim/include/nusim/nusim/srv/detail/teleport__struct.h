// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nusim:srv/Teleport.idl
// generated code does not contain a copyright notice

#ifndef NUSIM__SRV__DETAIL__TELEPORT__STRUCT_H_
#define NUSIM__SRV__DETAIL__TELEPORT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Teleport in the package nusim.
typedef struct nusim__srv__Teleport_Request
{
  double x;
  double y;
  double theta;
} nusim__srv__Teleport_Request;

// Struct for a sequence of nusim__srv__Teleport_Request.
typedef struct nusim__srv__Teleport_Request__Sequence
{
  nusim__srv__Teleport_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nusim__srv__Teleport_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Teleport in the package nusim.
typedef struct nusim__srv__Teleport_Response
{
  uint8_t structure_needs_at_least_one_member;
} nusim__srv__Teleport_Response;

// Struct for a sequence of nusim__srv__Teleport_Response.
typedef struct nusim__srv__Teleport_Response__Sequence
{
  nusim__srv__Teleport_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nusim__srv__Teleport_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  nusim__srv__Teleport_Event__request__MAX_SIZE = 1
};
// response
enum
{
  nusim__srv__Teleport_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Teleport in the package nusim.
typedef struct nusim__srv__Teleport_Event
{
  service_msgs__msg__ServiceEventInfo info;
  nusim__srv__Teleport_Request__Sequence request;
  nusim__srv__Teleport_Response__Sequence response;
} nusim__srv__Teleport_Event;

// Struct for a sequence of nusim__srv__Teleport_Event.
typedef struct nusim__srv__Teleport_Event__Sequence
{
  nusim__srv__Teleport_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nusim__srv__Teleport_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NUSIM__SRV__DETAIL__TELEPORT__STRUCT_H_
