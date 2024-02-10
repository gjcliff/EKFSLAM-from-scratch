// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nuturtle_control:msg/RobotConfiguration.idl
// generated code does not contain a copyright notice

#ifndef NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__BUILDER_HPP_
#define NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nuturtle_control/msg/detail/robot_configuration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nuturtle_control
{

namespace msg
{

namespace builder
{

class Init_RobotConfiguration_y
{
public:
  explicit Init_RobotConfiguration_y(::nuturtle_control::msg::RobotConfiguration & msg)
  : msg_(msg)
  {}
  ::nuturtle_control::msg::RobotConfiguration y(::nuturtle_control::msg::RobotConfiguration::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nuturtle_control::msg::RobotConfiguration msg_;
};

class Init_RobotConfiguration_x
{
public:
  explicit Init_RobotConfiguration_x(::nuturtle_control::msg::RobotConfiguration & msg)
  : msg_(msg)
  {}
  Init_RobotConfiguration_y x(::nuturtle_control::msg::RobotConfiguration::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RobotConfiguration_y(msg_);
  }

private:
  ::nuturtle_control::msg::RobotConfiguration msg_;
};

class Init_RobotConfiguration_theta
{
public:
  Init_RobotConfiguration_theta()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotConfiguration_x theta(::nuturtle_control::msg::RobotConfiguration::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_RobotConfiguration_x(msg_);
  }

private:
  ::nuturtle_control::msg::RobotConfiguration msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nuturtle_control::msg::RobotConfiguration>()
{
  return nuturtle_control::msg::builder::Init_RobotConfiguration_theta();
}

}  // namespace nuturtle_control

#endif  // NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__BUILDER_HPP_
