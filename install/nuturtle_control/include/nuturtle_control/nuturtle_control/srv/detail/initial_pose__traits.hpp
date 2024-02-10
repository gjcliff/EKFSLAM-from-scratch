// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nuturtle_control:srv/InitialPose.idl
// generated code does not contain a copyright notice

#ifndef NUTURTLE_CONTROL__SRV__DETAIL__INITIAL_POSE__TRAITS_HPP_
#define NUTURTLE_CONTROL__SRV__DETAIL__INITIAL_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nuturtle_control/srv/detail/initial_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'q'
#include "nuturtle_control/msg/detail/robot_configuration__traits.hpp"

namespace nuturtle_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const InitialPose_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: q
  {
    out << "q: ";
    to_flow_style_yaml(msg.q, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InitialPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: q
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q:\n";
    to_block_style_yaml(msg.q, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InitialPose_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nuturtle_control

namespace rosidl_generator_traits
{

[[deprecated("use nuturtle_control::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nuturtle_control::srv::InitialPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  nuturtle_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nuturtle_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const nuturtle_control::srv::InitialPose_Request & msg)
{
  return nuturtle_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nuturtle_control::srv::InitialPose_Request>()
{
  return "nuturtle_control::srv::InitialPose_Request";
}

template<>
inline const char * name<nuturtle_control::srv::InitialPose_Request>()
{
  return "nuturtle_control/srv/InitialPose_Request";
}

template<>
struct has_fixed_size<nuturtle_control::srv::InitialPose_Request>
  : std::integral_constant<bool, has_fixed_size<nuturtle_control::msg::RobotConfiguration>::value> {};

template<>
struct has_bounded_size<nuturtle_control::srv::InitialPose_Request>
  : std::integral_constant<bool, has_bounded_size<nuturtle_control::msg::RobotConfiguration>::value> {};

template<>
struct is_message<nuturtle_control::srv::InitialPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace nuturtle_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const InitialPose_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InitialPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InitialPose_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nuturtle_control

namespace rosidl_generator_traits
{

[[deprecated("use nuturtle_control::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nuturtle_control::srv::InitialPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  nuturtle_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nuturtle_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const nuturtle_control::srv::InitialPose_Response & msg)
{
  return nuturtle_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nuturtle_control::srv::InitialPose_Response>()
{
  return "nuturtle_control::srv::InitialPose_Response";
}

template<>
inline const char * name<nuturtle_control::srv::InitialPose_Response>()
{
  return "nuturtle_control/srv/InitialPose_Response";
}

template<>
struct has_fixed_size<nuturtle_control::srv::InitialPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<nuturtle_control::srv::InitialPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<nuturtle_control::srv::InitialPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace nuturtle_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const InitialPose_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InitialPose_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InitialPose_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nuturtle_control

namespace rosidl_generator_traits
{

[[deprecated("use nuturtle_control::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nuturtle_control::srv::InitialPose_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  nuturtle_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nuturtle_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const nuturtle_control::srv::InitialPose_Event & msg)
{
  return nuturtle_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nuturtle_control::srv::InitialPose_Event>()
{
  return "nuturtle_control::srv::InitialPose_Event";
}

template<>
inline const char * name<nuturtle_control::srv::InitialPose_Event>()
{
  return "nuturtle_control/srv/InitialPose_Event";
}

template<>
struct has_fixed_size<nuturtle_control::srv::InitialPose_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nuturtle_control::srv::InitialPose_Event>
  : std::integral_constant<bool, has_bounded_size<nuturtle_control::srv::InitialPose_Request>::value && has_bounded_size<nuturtle_control::srv::InitialPose_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<nuturtle_control::srv::InitialPose_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nuturtle_control::srv::InitialPose>()
{
  return "nuturtle_control::srv::InitialPose";
}

template<>
inline const char * name<nuturtle_control::srv::InitialPose>()
{
  return "nuturtle_control/srv/InitialPose";
}

template<>
struct has_fixed_size<nuturtle_control::srv::InitialPose>
  : std::integral_constant<
    bool,
    has_fixed_size<nuturtle_control::srv::InitialPose_Request>::value &&
    has_fixed_size<nuturtle_control::srv::InitialPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<nuturtle_control::srv::InitialPose>
  : std::integral_constant<
    bool,
    has_bounded_size<nuturtle_control::srv::InitialPose_Request>::value &&
    has_bounded_size<nuturtle_control::srv::InitialPose_Response>::value
  >
{
};

template<>
struct is_service<nuturtle_control::srv::InitialPose>
  : std::true_type
{
};

template<>
struct is_service_request<nuturtle_control::srv::InitialPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<nuturtle_control::srv::InitialPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // NUTURTLE_CONTROL__SRV__DETAIL__INITIAL_POSE__TRAITS_HPP_
