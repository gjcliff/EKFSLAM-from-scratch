// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nuturtle_control:srv/InitialPose.idl
// generated code does not contain a copyright notice

#ifndef NUTURTLE_CONTROL__SRV__DETAIL__INITIAL_POSE__BUILDER_HPP_
#define NUTURTLE_CONTROL__SRV__DETAIL__INITIAL_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nuturtle_control/srv/detail/initial_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nuturtle_control
{

namespace srv
{

namespace builder
{

class Init_InitialPose_Request_q
{
public:
  Init_InitialPose_Request_q()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::nuturtle_control::srv::InitialPose_Request q(::nuturtle_control::srv::InitialPose_Request::_q_type arg)
  {
    msg_.q = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nuturtle_control::srv::InitialPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nuturtle_control::srv::InitialPose_Request>()
{
  return nuturtle_control::srv::builder::Init_InitialPose_Request_q();
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
auto build<::nuturtle_control::srv::InitialPose_Response>()
{
  return ::nuturtle_control::srv::InitialPose_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace nuturtle_control


namespace nuturtle_control
{

namespace srv
{

namespace builder
{

class Init_InitialPose_Event_response
{
public:
  explicit Init_InitialPose_Event_response(::nuturtle_control::srv::InitialPose_Event & msg)
  : msg_(msg)
  {}
  ::nuturtle_control::srv::InitialPose_Event response(::nuturtle_control::srv::InitialPose_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nuturtle_control::srv::InitialPose_Event msg_;
};

class Init_InitialPose_Event_request
{
public:
  explicit Init_InitialPose_Event_request(::nuturtle_control::srv::InitialPose_Event & msg)
  : msg_(msg)
  {}
  Init_InitialPose_Event_response request(::nuturtle_control::srv::InitialPose_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_InitialPose_Event_response(msg_);
  }

private:
  ::nuturtle_control::srv::InitialPose_Event msg_;
};

class Init_InitialPose_Event_info
{
public:
  Init_InitialPose_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InitialPose_Event_request info(::nuturtle_control::srv::InitialPose_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_InitialPose_Event_request(msg_);
  }

private:
  ::nuturtle_control::srv::InitialPose_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nuturtle_control::srv::InitialPose_Event>()
{
  return nuturtle_control::srv::builder::Init_InitialPose_Event_info();
}

}  // namespace nuturtle_control

#endif  // NUTURTLE_CONTROL__SRV__DETAIL__INITIAL_POSE__BUILDER_HPP_
