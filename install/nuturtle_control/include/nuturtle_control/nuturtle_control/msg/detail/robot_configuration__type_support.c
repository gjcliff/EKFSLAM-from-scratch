// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from nuturtle_control:msg/RobotConfiguration.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "nuturtle_control/msg/detail/robot_configuration__rosidl_typesupport_introspection_c.h"
#include "nuturtle_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "nuturtle_control/msg/detail/robot_configuration__functions.h"
#include "nuturtle_control/msg/detail/robot_configuration__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  nuturtle_control__msg__RobotConfiguration__init(message_memory);
}

void nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_fini_function(void * message_memory)
{
  nuturtle_control__msg__RobotConfiguration__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_message_member_array[3] = {
  {
    "theta",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nuturtle_control__msg__RobotConfiguration, theta),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nuturtle_control__msg__RobotConfiguration, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nuturtle_control__msg__RobotConfiguration, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_message_members = {
  "nuturtle_control__msg",  // message namespace
  "RobotConfiguration",  // message name
  3,  // number of fields
  sizeof(nuturtle_control__msg__RobotConfiguration),
  nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_message_member_array,  // message members
  nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_init_function,  // function to initialize message memory (memory has to be allocated)
  nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_message_type_support_handle = {
  0,
  &nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_message_members,
  get_message_typesupport_handle_function,
  &nuturtle_control__msg__RobotConfiguration__get_type_hash,
  &nuturtle_control__msg__RobotConfiguration__get_type_description,
  &nuturtle_control__msg__RobotConfiguration__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nuturtle_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nuturtle_control, msg, RobotConfiguration)() {
  if (!nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_message_type_support_handle.typesupport_identifier) {
    nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &nuturtle_control__msg__RobotConfiguration__rosidl_typesupport_introspection_c__RobotConfiguration_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
