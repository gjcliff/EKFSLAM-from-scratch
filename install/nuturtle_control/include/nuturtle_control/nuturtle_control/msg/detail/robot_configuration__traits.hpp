// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nuturtle_control:msg/RobotConfiguration.idl
// generated code does not contain a copyright notice

#ifndef NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__TRAITS_HPP_
#define NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nuturtle_control/msg/detail/robot_configuration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace nuturtle_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotConfiguration & msg,
  std::ostream & out)
{
  out << "{";
  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotConfiguration & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotConfiguration & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace nuturtle_control

namespace rosidl_generator_traits
{

[[deprecated("use nuturtle_control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nuturtle_control::msg::RobotConfiguration & msg,
  std::ostream & out, size_t indentation = 0)
{
  nuturtle_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nuturtle_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const nuturtle_control::msg::RobotConfiguration & msg)
{
  return nuturtle_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<nuturtle_control::msg::RobotConfiguration>()
{
  return "nuturtle_control::msg::RobotConfiguration";
}

template<>
inline const char * name<nuturtle_control::msg::RobotConfiguration>()
{
  return "nuturtle_control/msg/RobotConfiguration";
}

template<>
struct has_fixed_size<nuturtle_control::msg::RobotConfiguration>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<nuturtle_control::msg::RobotConfiguration>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<nuturtle_control::msg::RobotConfiguration>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__TRAITS_HPP_
