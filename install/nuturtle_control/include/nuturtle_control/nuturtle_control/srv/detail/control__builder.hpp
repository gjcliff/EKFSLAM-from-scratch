// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nuturtle_control:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef NUTURTLE_CONTROL__SRV__DETAIL__CONTROL__BUILDER_HPP_
#define NUTURTLE_CONTROL__SRV__DETAIL__CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nuturtle_control/srv/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nuturtle_control
{

namespace srv
{

namespace builder
{

class Init_Control_Request_radius
{
public:
  explicit Init_Control_Request_radius(::nuturtle_control::srv::Control_Request & msg)
  : msg_(msg)
  {}
  ::nuturtle_control::srv::Control_Request radius(::nuturtle_control::srv::Control_Request::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nuturtle_control::srv::Control_Request msg_;
};

class Init_Control_Request_velocity
{
public:
  Init_Control_Request_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_Request_radius velocity(::nuturtle_control::srv::Control_Request::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Control_Request_radius(msg_);
  }

private:
  ::nuturtle_control::srv::Control_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nuturtle_control::srv::Control_Request>()
{
  return nuturtle_control::srv::builder::Init_Control_Request_velocity();
}

}  // namespace nuturtle_control


namespace nuturtle_control
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nuturtle_control::srv::Control_Response>()
{
  return ::nuturtle_control::srv::Control_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace nuturtle_control


namespace nuturtle_control
{

namespace srv
{

namespace builder
{

class Init_Control_Event_response
{
public:
  explicit Init_Control_Event_response(::nuturtle_control::srv::Control_Event & msg)
  : msg_(msg)
  {}
  ::nuturtle_control::srv::Control_Event response(::nuturtle_control::srv::Control_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nuturtle_control::srv::Control_Event msg_;
};

class Init_Control_Event_request
{
public:
  explicit Init_Control_Event_request(::nuturtle_control::srv::Control_Event & msg)
  : msg_(msg)
  {}
  Init_Control_Event_response request(::nuturtle_control::srv::Control_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Control_Event_response(msg_);
  }

private:
  ::nuturtle_control::srv::Control_Event msg_;
};

class Init_Control_Event_info
{
public:
  Init_Control_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_Event_request info(::nuturtle_control::srv::Control_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Control_Event_request(msg_);
  }

private:
  ::nuturtle_control::srv::Control_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nuturtle_control::srv::Control_Event>()
{
  return nuturtle_control::srv::builder::Init_Control_Event_info();
}

}  // namespace nuturtle_control

#endif  // NUTURTLE_CONTROL__SRV__DETAIL__CONTROL__BUILDER_HPP_
