// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nusim:srv/Teleport.idl
// generated code does not contain a copyright notice

#ifndef NUSIM__SRV__DETAIL__TELEPORT__STRUCT_HPP_
#define NUSIM__SRV__DETAIL__TELEPORT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__nusim__srv__Teleport_Request __attribute__((deprecated))
#else
# define DEPRECATED__nusim__srv__Teleport_Request __declspec(deprecated)
#endif

namespace nusim
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Teleport_Request_
{
  using Type = Teleport_Request_<ContainerAllocator>;

  explicit Teleport_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  explicit Teleport_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _theta_type =
    double;
  _theta_type theta;

  // setters for named parameter idiom
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
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nusim::srv::Teleport_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const nusim::srv::Teleport_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nusim::srv::Teleport_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nusim::srv::Teleport_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nusim::srv::Teleport_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nusim::srv::Teleport_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nusim::srv::Teleport_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nusim::srv::Teleport_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nusim::srv::Teleport_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nusim::srv::Teleport_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nusim__srv__Teleport_Request
    std::shared_ptr<nusim::srv::Teleport_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nusim__srv__Teleport_Request
    std::shared_ptr<nusim::srv::Teleport_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Teleport_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    return true;
  }
  bool operator!=(const Teleport_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Teleport_Request_

// alias to use template instance with default allocator
using Teleport_Request =
  nusim::srv::Teleport_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace nusim


#ifndef _WIN32
# define DEPRECATED__nusim__srv__Teleport_Response __attribute__((deprecated))
#else
# define DEPRECATED__nusim__srv__Teleport_Response __declspec(deprecated)
#endif

namespace nusim
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Teleport_Response_
{
  using Type = Teleport_Response_<ContainerAllocator>;

  explicit Teleport_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Teleport_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    nusim::srv::Teleport_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const nusim::srv::Teleport_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nusim::srv::Teleport_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nusim::srv::Teleport_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nusim::srv::Teleport_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nusim::srv::Teleport_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nusim::srv::Teleport_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nusim::srv::Teleport_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nusim::srv::Teleport_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nusim::srv::Teleport_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nusim__srv__Teleport_Response
    std::shared_ptr<nusim::srv::Teleport_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nusim__srv__Teleport_Response
    std::shared_ptr<nusim::srv::Teleport_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Teleport_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Teleport_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Teleport_Response_

// alias to use template instance with default allocator
using Teleport_Response =
  nusim::srv::Teleport_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace nusim


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nusim__srv__Teleport_Event __attribute__((deprecated))
#else
# define DEPRECATED__nusim__srv__Teleport_Event __declspec(deprecated)
#endif

namespace nusim
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Teleport_Event_
{
  using Type = Teleport_Event_<ContainerAllocator>;

  explicit Teleport_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Teleport_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<nusim::srv::Teleport_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nusim::srv::Teleport_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<nusim::srv::Teleport_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nusim::srv::Teleport_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<nusim::srv::Teleport_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nusim::srv::Teleport_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<nusim::srv::Teleport_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nusim::srv::Teleport_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nusim::srv::Teleport_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const nusim::srv::Teleport_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nusim::srv::Teleport_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nusim::srv::Teleport_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nusim::srv::Teleport_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nusim::srv::Teleport_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nusim::srv::Teleport_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nusim::srv::Teleport_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nusim::srv::Teleport_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nusim::srv::Teleport_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nusim__srv__Teleport_Event
    std::shared_ptr<nusim::srv::Teleport_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nusim__srv__Teleport_Event
    std::shared_ptr<nusim::srv::Teleport_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Teleport_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Teleport_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Teleport_Event_

// alias to use template instance with default allocator
using Teleport_Event =
  nusim::srv::Teleport_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace nusim

namespace nusim
{

namespace srv
{

struct Teleport
{
  using Request = nusim::srv::Teleport_Request;
  using Response = nusim::srv::Teleport_Response;
  using Event = nusim::srv::Teleport_Event;
};

}  // namespace srv

}  // namespace nusim

#endif  // NUSIM__SRV__DETAIL__TELEPORT__STRUCT_HPP_
