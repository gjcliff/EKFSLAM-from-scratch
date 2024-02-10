// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nuturtle_control:msg/RobotConfiguration.idl
// generated code does not contain a copyright notice

#ifndef NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__STRUCT_HPP_
#define NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__nuturtle_control__msg__RobotConfiguration __attribute__((deprecated))
#else
# define DEPRECATED__nuturtle_control__msg__RobotConfiguration __declspec(deprecated)
#endif

namespace nuturtle_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotConfiguration_
{
  using Type = RobotConfiguration_<ContainerAllocator>;

  explicit RobotConfiguration_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->theta = 0.0;
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit RobotConfiguration_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->theta = 0.0;
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _theta_type =
    double;
  _theta_type theta;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nuturtle_control::msg::RobotConfiguration_<ContainerAllocator> *;
  using ConstRawPtr =
    const nuturtle_control::msg::RobotConfiguration_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nuturtle_control::msg::RobotConfiguration_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nuturtle_control::msg::RobotConfiguration_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nuturtle_control::msg::RobotConfiguration_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nuturtle_control::msg::RobotConfiguration_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nuturtle_control::msg::RobotConfiguration_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nuturtle_control::msg::RobotConfiguration_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nuturtle_control::msg::RobotConfiguration_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nuturtle_control::msg::RobotConfiguration_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nuturtle_control__msg__RobotConfiguration
    std::shared_ptr<nuturtle_control::msg::RobotConfiguration_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nuturtle_control__msg__RobotConfiguration
    std::shared_ptr<nuturtle_control::msg::RobotConfiguration_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotConfiguration_ & other) const
  {
    if (this->theta != other.theta) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotConfiguration_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotConfiguration_

// alias to use template instance with default allocator
using RobotConfiguration =
  nuturtle_control::msg::RobotConfiguration_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nuturtle_control

#endif  // NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__STRUCT_HPP_
