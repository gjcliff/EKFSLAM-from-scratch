// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nusim:srv/Teleport.idl
// generated code does not contain a copyright notice

#ifndef NUSIM__SRV__DETAIL__TELEPORT__BUILDER_HPP_
#define NUSIM__SRV__DETAIL__TELEPORT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nusim/srv/detail/teleport__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nusim
{

namespace srv
{

namespace builder
{

class Init_Teleport_Request_theta
{
public:
  explicit Init_Teleport_Request_theta(::nusim::srv::Teleport_Request & msg)
  : msg_(msg)
  {}
  ::nusim::srv::Teleport_Request theta(::nusim::srv::Teleport_Request::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nusim::srv::Teleport_Request msg_;
};

class Init_Teleport_Request_y
{
public:
  explicit Init_Teleport_Request_y(::nusim::srv::Teleport_Request & msg)
  : msg_(msg)
  {}
  Init_Teleport_Request_theta y(::nusim::srv::Teleport_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Teleport_Request_theta(msg_);
  }

private:
  ::nusim::srv::Teleport_Request msg_;
};

class Init_Teleport_Request_x
{
public:
  Init_Teleport_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Teleport_Request_y x(::nusim::srv::Teleport_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Teleport_Request_y(msg_);
  }

private:
  ::nusim::srv::Teleport_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nusim::srv::Teleport_Request>()
{
  return nusim::srv::builder::Init_Teleport_Request_x();
}

}  // namespace nusim


namespace nusim
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nusim::srv::Teleport_Response>()
{
  return ::nusim::srv::Teleport_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace nusim


namespace nusim
{

namespace srv
{

namespace builder
{

class Init_Teleport_Event_response
{
public:
  explicit Init_Teleport_Event_response(::nusim::srv::Teleport_Event & msg)
  : msg_(msg)
  {}
  ::nusim::srv::Teleport_Event response(::nusim::srv::Teleport_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nusim::srv::Teleport_Event msg_;
};

class Init_Teleport_Event_request
{
public:
  explicit Init_Teleport_Event_request(::nusim::srv::Teleport_Event & msg)
  : msg_(msg)
  {}
  Init_Teleport_Event_response request(::nusim::srv::Teleport_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Teleport_Event_response(msg_);
  }

private:
  ::nusim::srv::Teleport_Event msg_;
};

class Init_Teleport_Event_info
{
public:
  Init_Teleport_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Teleport_Event_request info(::nusim::srv::Teleport_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Teleport_Event_request(msg_);
  }

private:
  ::nusim::srv::Teleport_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nusim::srv::Teleport_Event>()
{
  return nusim::srv::builder::Init_Teleport_Event_info();
}

}  // namespace nusim

#endif  // NUSIM__SRV__DETAIL__TELEPORT__BUILDER_HPP_
