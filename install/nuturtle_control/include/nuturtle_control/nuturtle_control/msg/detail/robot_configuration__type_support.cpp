// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from nuturtle_control:msg/RobotConfiguration.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "nuturtle_control/msg/detail/robot_configuration__functions.h"
#include "nuturtle_control/msg/detail/robot_configuration__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace nuturtle_control
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RobotConfiguration_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) nuturtle_control::msg::RobotConfiguration(_init);
}

void RobotConfiguration_fini_function(void * message_memory)
{
  auto typed_message = static_cast<nuturtle_control::msg::RobotConfiguration *>(message_memory);
  typed_message->~RobotConfiguration();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RobotConfiguration_message_member_array[3] = {
  {
    "theta",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nuturtle_control::msg::RobotConfiguration, theta),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nuturtle_control::msg::RobotConfiguration, x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nuturtle_control::msg::RobotConfiguration, y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RobotConfiguration_message_members = {
  "nuturtle_control::msg",  // message namespace
  "RobotConfiguration",  // message name
  3,  // number of fields
  sizeof(nuturtle_control::msg::RobotConfiguration),
  RobotConfiguration_message_member_array,  // message members
  RobotConfiguration_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotConfiguration_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RobotConfiguration_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RobotConfiguration_message_members,
  get_message_typesupport_handle_function,
  &nuturtle_control__msg__RobotConfiguration__get_type_hash,
  &nuturtle_control__msg__RobotConfiguration__get_type_description,
  &nuturtle_control__msg__RobotConfiguration__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace nuturtle_control


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nuturtle_control::msg::RobotConfiguration>()
{
  return &::nuturtle_control::msg::rosidl_typesupport_introspection_cpp::RobotConfiguration_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nuturtle_control, msg, RobotConfiguration)() {
  return &::nuturtle_control::msg::rosidl_typesupport_introspection_cpp::RobotConfiguration_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
