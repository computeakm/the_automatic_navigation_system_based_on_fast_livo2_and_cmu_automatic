// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav2_msgs:srv/DynamicEdges.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__STRUCT_HPP_
#define NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'adjust_edges'
#include "nav2_msgs/msg/detail/edge_cost__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__srv__DynamicEdges_Request __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__srv__DynamicEdges_Request __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DynamicEdges_Request_
{
  using Type = DynamicEdges_Request_<ContainerAllocator>;

  explicit DynamicEdges_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit DynamicEdges_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _closed_edges_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _closed_edges_type closed_edges;
  using _opened_edges_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _opened_edges_type opened_edges;
  using _adjust_edges_type =
    std::vector<nav2_msgs::msg::EdgeCost_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav2_msgs::msg::EdgeCost_<ContainerAllocator>>>;
  _adjust_edges_type adjust_edges;

  // setters for named parameter idiom
  Type & set__closed_edges(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->closed_edges = _arg;
    return *this;
  }
  Type & set__opened_edges(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->opened_edges = _arg;
    return *this;
  }
  Type & set__adjust_edges(
    const std::vector<nav2_msgs::msg::EdgeCost_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav2_msgs::msg::EdgeCost_<ContainerAllocator>>> & _arg)
  {
    this->adjust_edges = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__srv__DynamicEdges_Request
    std::shared_ptr<nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__srv__DynamicEdges_Request
    std::shared_ptr<nav2_msgs::srv::DynamicEdges_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DynamicEdges_Request_ & other) const
  {
    if (this->closed_edges != other.closed_edges) {
      return false;
    }
    if (this->opened_edges != other.opened_edges) {
      return false;
    }
    if (this->adjust_edges != other.adjust_edges) {
      return false;
    }
    return true;
  }
  bool operator!=(const DynamicEdges_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DynamicEdges_Request_

// alias to use template instance with default allocator
using DynamicEdges_Request =
  nav2_msgs::srv::DynamicEdges_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace nav2_msgs


#ifndef _WIN32
# define DEPRECATED__nav2_msgs__srv__DynamicEdges_Response __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__srv__DynamicEdges_Response __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DynamicEdges_Response_
{
  using Type = DynamicEdges_Response_<ContainerAllocator>;

  explicit DynamicEdges_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit DynamicEdges_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__srv__DynamicEdges_Response
    std::shared_ptr<nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__srv__DynamicEdges_Response
    std::shared_ptr<nav2_msgs::srv::DynamicEdges_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DynamicEdges_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const DynamicEdges_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DynamicEdges_Response_

// alias to use template instance with default allocator
using DynamicEdges_Response =
  nav2_msgs::srv::DynamicEdges_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace nav2_msgs

namespace nav2_msgs
{

namespace srv
{

struct DynamicEdges
{
  using Request = nav2_msgs::srv::DynamicEdges_Request;
  using Response = nav2_msgs::srv::DynamicEdges_Response;
};

}  // namespace srv

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__STRUCT_HPP_
