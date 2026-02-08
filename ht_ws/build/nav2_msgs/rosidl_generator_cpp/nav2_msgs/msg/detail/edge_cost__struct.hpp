// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav2_msgs:msg/EdgeCost.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__EDGE_COST__STRUCT_HPP_
#define NAV2_MSGS__MSG__DETAIL__EDGE_COST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__nav2_msgs__msg__EdgeCost __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__msg__EdgeCost __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EdgeCost_
{
  using Type = EdgeCost_<ContainerAllocator>;

  explicit EdgeCost_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->edgeid = 0;
      this->cost = 0.0f;
    }
  }

  explicit EdgeCost_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->edgeid = 0;
      this->cost = 0.0f;
    }
  }

  // field types and members
  using _edgeid_type =
    uint16_t;
  _edgeid_type edgeid;
  using _cost_type =
    float;
  _cost_type cost;

  // setters for named parameter idiom
  Type & set__edgeid(
    const uint16_t & _arg)
  {
    this->edgeid = _arg;
    return *this;
  }
  Type & set__cost(
    const float & _arg)
  {
    this->cost = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::msg::EdgeCost_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::msg::EdgeCost_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::msg::EdgeCost_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::msg::EdgeCost_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::msg::EdgeCost_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::msg::EdgeCost_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::msg::EdgeCost_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::msg::EdgeCost_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::msg::EdgeCost_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::msg::EdgeCost_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__msg__EdgeCost
    std::shared_ptr<nav2_msgs::msg::EdgeCost_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__msg__EdgeCost
    std::shared_ptr<nav2_msgs::msg::EdgeCost_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EdgeCost_ & other) const
  {
    if (this->edgeid != other.edgeid) {
      return false;
    }
    if (this->cost != other.cost) {
      return false;
    }
    return true;
  }
  bool operator!=(const EdgeCost_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EdgeCost_

// alias to use template instance with default allocator
using EdgeCost =
  nav2_msgs::msg::EdgeCost_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__MSG__DETAIL__EDGE_COST__STRUCT_HPP_
