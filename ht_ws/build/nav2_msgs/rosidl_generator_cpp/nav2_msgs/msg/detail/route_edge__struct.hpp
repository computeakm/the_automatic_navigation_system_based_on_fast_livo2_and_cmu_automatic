// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav2_msgs:msg/RouteEdge.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__ROUTE_EDGE__STRUCT_HPP_
#define NAV2_MSGS__MSG__DETAIL__ROUTE_EDGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'start'
// Member 'end'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__msg__RouteEdge __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__msg__RouteEdge __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RouteEdge_
{
  using Type = RouteEdge_<ContainerAllocator>;

  explicit RouteEdge_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_init),
    end(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->edgeid = 0;
    }
  }

  explicit RouteEdge_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_alloc, _init),
    end(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->edgeid = 0;
    }
  }

  // field types and members
  using _edgeid_type =
    uint16_t;
  _edgeid_type edgeid;
  using _start_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _start_type start;
  using _end_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _end_type end;

  // setters for named parameter idiom
  Type & set__edgeid(
    const uint16_t & _arg)
  {
    this->edgeid = _arg;
    return *this;
  }
  Type & set__start(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__end(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->end = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::msg::RouteEdge_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::msg::RouteEdge_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::msg::RouteEdge_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::msg::RouteEdge_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::msg::RouteEdge_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::msg::RouteEdge_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::msg::RouteEdge_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::msg::RouteEdge_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::msg::RouteEdge_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::msg::RouteEdge_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__msg__RouteEdge
    std::shared_ptr<nav2_msgs::msg::RouteEdge_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__msg__RouteEdge
    std::shared_ptr<nav2_msgs::msg::RouteEdge_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RouteEdge_ & other) const
  {
    if (this->edgeid != other.edgeid) {
      return false;
    }
    if (this->start != other.start) {
      return false;
    }
    if (this->end != other.end) {
      return false;
    }
    return true;
  }
  bool operator!=(const RouteEdge_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RouteEdge_

// alias to use template instance with default allocator
using RouteEdge =
  nav2_msgs::msg::RouteEdge_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__MSG__DETAIL__ROUTE_EDGE__STRUCT_HPP_
