// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from nuturtle_control:msg/RobotConfiguration.idl
// generated code does not contain a copyright notice

#ifndef NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "nuturtle_control/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "nuturtle_control/msg/detail/robot_configuration__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace nuturtle_control
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nuturtle_control
cdr_serialize(
  const nuturtle_control::msg::RobotConfiguration & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nuturtle_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  nuturtle_control::msg::RobotConfiguration & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nuturtle_control
get_serialized_size(
  const nuturtle_control::msg::RobotConfiguration & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nuturtle_control
max_serialized_size_RobotConfiguration(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace nuturtle_control

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nuturtle_control
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nuturtle_control, msg, RobotConfiguration)();

#ifdef __cplusplus
}
#endif

#endif  // NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
