// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav2_msgs:action/ComputeAndTrackRoute.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__ACTION__DETAIL__COMPUTE_AND_TRACK_ROUTE__STRUCT_HPP_
#define NAV2_MSGS__ACTION__DETAIL__COMPUTE_AND_TRACK_ROUTE__STRUCT_HPP_

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
// Member 'goal'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Goal __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Goal __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ComputeAndTrackRoute_Goal_
{
  using Type = ComputeAndTrackRoute_Goal_<ContainerAllocator>;

  explicit ComputeAndTrackRoute_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_init),
    goal(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start_id = 0;
      this->goal_id = 0;
      this->use_start = false;
      this->use_poses = false;
    }
  }

  explicit ComputeAndTrackRoute_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_alloc, _init),
    goal(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start_id = 0;
      this->goal_id = 0;
      this->use_start = false;
      this->use_poses = false;
    }
  }

  // field types and members
  using _start_id_type =
    uint16_t;
  _start_id_type start_id;
  using _start_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _start_type start;
  using _goal_id_type =
    uint16_t;
  _goal_id_type goal_id;
  using _goal_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _goal_type goal;
  using _use_start_type =
    bool;
  _use_start_type use_start;
  using _use_poses_type =
    bool;
  _use_poses_type use_poses;

  // setters for named parameter idiom
  Type & set__start_id(
    const uint16_t & _arg)
  {
    this->start_id = _arg;
    return *this;
  }
  Type & set__start(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__goal_id(
    const uint16_t & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }
  Type & set__use_start(
    const bool & _arg)
  {
    this->use_start = _arg;
    return *this;
  }
  Type & set__use_poses(
    const bool & _arg)
  {
    this->use_poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Goal
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Goal
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeAndTrackRoute_Goal_ & other) const
  {
    if (this->start_id != other.start_id) {
      return false;
    }
    if (this->start != other.start) {
      return false;
    }
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    if (this->use_start != other.use_start) {
      return false;
    }
    if (this->use_poses != other.use_poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeAndTrackRoute_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeAndTrackRoute_Goal_

// alias to use template instance with default allocator
using ComputeAndTrackRoute_Goal =
  nav2_msgs::action::ComputeAndTrackRoute_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace nav2_msgs


// Include directives for member types
// Member 'execution_duration'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Result __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Result __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ComputeAndTrackRoute_Result_
{
  using Type = ComputeAndTrackRoute_Result_<ContainerAllocator>;

  explicit ComputeAndTrackRoute_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : execution_duration(_init)
  {
    (void)_init;
  }

  explicit ComputeAndTrackRoute_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : execution_duration(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _execution_duration_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _execution_duration_type execution_duration;

  // setters for named parameter idiom
  Type & set__execution_duration(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->execution_duration = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t NONE =
    0u;
  static constexpr uint16_t UNKNOWN =
    400u;
  static constexpr uint16_t TF_ERROR =
    401u;
  static constexpr uint16_t NO_VALID_GRAPH =
    402u;
  static constexpr uint16_t INDETERMINANT_NODES_ON_GRAPH =
    403u;
  static constexpr uint16_t TIMEOUT =
    404u;
  static constexpr uint16_t NO_VALID_ROUTE =
    405u;
  static constexpr uint16_t OPERATION_FAILED =
    406u;
  static constexpr uint16_t INVALID_EDGE_SCORER_USE =
    407u;

  // pointer types
  using RawPtr =
    nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Result
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Result
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeAndTrackRoute_Result_ & other) const
  {
    if (this->execution_duration != other.execution_duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeAndTrackRoute_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeAndTrackRoute_Result_

// alias to use template instance with default allocator
using ComputeAndTrackRoute_Result =
  nav2_msgs::action::ComputeAndTrackRoute_Result_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ComputeAndTrackRoute_Result_<ContainerAllocator>::NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ComputeAndTrackRoute_Result_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ComputeAndTrackRoute_Result_<ContainerAllocator>::TF_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ComputeAndTrackRoute_Result_<ContainerAllocator>::NO_VALID_GRAPH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ComputeAndTrackRoute_Result_<ContainerAllocator>::INDETERMINANT_NODES_ON_GRAPH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ComputeAndTrackRoute_Result_<ContainerAllocator>::TIMEOUT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ComputeAndTrackRoute_Result_<ContainerAllocator>::NO_VALID_ROUTE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ComputeAndTrackRoute_Result_<ContainerAllocator>::OPERATION_FAILED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ComputeAndTrackRoute_Result_<ContainerAllocator>::INVALID_EDGE_SCORER_USE;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace nav2_msgs


// Include directives for member types
// Member 'route'
#include "nav2_msgs/msg/detail/route__struct.hpp"
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Feedback __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ComputeAndTrackRoute_Feedback_
{
  using Type = ComputeAndTrackRoute_Feedback_<ContainerAllocator>;

  explicit ComputeAndTrackRoute_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : route(_init),
    path(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->last_node_id = 0;
      this->next_node_id = 0;
      this->current_edge_id = 0;
      this->rerouted = false;
    }
  }

  explicit ComputeAndTrackRoute_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : route(_alloc, _init),
    path(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->last_node_id = 0;
      this->next_node_id = 0;
      this->current_edge_id = 0;
      this->rerouted = false;
    }
  }

  // field types and members
  using _last_node_id_type =
    uint16_t;
  _last_node_id_type last_node_id;
  using _next_node_id_type =
    uint16_t;
  _next_node_id_type next_node_id;
  using _current_edge_id_type =
    uint16_t;
  _current_edge_id_type current_edge_id;
  using _route_type =
    nav2_msgs::msg::Route_<ContainerAllocator>;
  _route_type route;
  using _path_type =
    nav_msgs::msg::Path_<ContainerAllocator>;
  _path_type path;
  using _operations_triggered_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _operations_triggered_type operations_triggered;
  using _rerouted_type =
    bool;
  _rerouted_type rerouted;

  // setters for named parameter idiom
  Type & set__last_node_id(
    const uint16_t & _arg)
  {
    this->last_node_id = _arg;
    return *this;
  }
  Type & set__next_node_id(
    const uint16_t & _arg)
  {
    this->next_node_id = _arg;
    return *this;
  }
  Type & set__current_edge_id(
    const uint16_t & _arg)
  {
    this->current_edge_id = _arg;
    return *this;
  }
  Type & set__route(
    const nav2_msgs::msg::Route_<ContainerAllocator> & _arg)
  {
    this->route = _arg;
    return *this;
  }
  Type & set__path(
    const nav_msgs::msg::Path_<ContainerAllocator> & _arg)
  {
    this->path = _arg;
    return *this;
  }
  Type & set__operations_triggered(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->operations_triggered = _arg;
    return *this;
  }
  Type & set__rerouted(
    const bool & _arg)
  {
    this->rerouted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Feedback
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_Feedback
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeAndTrackRoute_Feedback_ & other) const
  {
    if (this->last_node_id != other.last_node_id) {
      return false;
    }
    if (this->next_node_id != other.next_node_id) {
      return false;
    }
    if (this->current_edge_id != other.current_edge_id) {
      return false;
    }
    if (this->route != other.route) {
      return false;
    }
    if (this->path != other.path) {
      return false;
    }
    if (this->operations_triggered != other.operations_triggered) {
      return false;
    }
    if (this->rerouted != other.rerouted) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeAndTrackRoute_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeAndTrackRoute_Feedback_

// alias to use template instance with default allocator
using ComputeAndTrackRoute_Feedback =
  nav2_msgs::action::ComputeAndTrackRoute_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace nav2_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "nav2_msgs/action/detail/compute_and_track_route__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ComputeAndTrackRoute_SendGoal_Request_
{
  using Type = ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator>;

  explicit ComputeAndTrackRoute_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit ComputeAndTrackRoute_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const nav2_msgs::action::ComputeAndTrackRoute_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Request
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeAndTrackRoute_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeAndTrackRoute_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeAndTrackRoute_SendGoal_Request_

// alias to use template instance with default allocator
using ComputeAndTrackRoute_SendGoal_Request =
  nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace nav2_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ComputeAndTrackRoute_SendGoal_Response_
{
  using Type = ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator>;

  explicit ComputeAndTrackRoute_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit ComputeAndTrackRoute_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_SendGoal_Response
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeAndTrackRoute_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeAndTrackRoute_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeAndTrackRoute_SendGoal_Response_

// alias to use template instance with default allocator
using ComputeAndTrackRoute_SendGoal_Response =
  nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace nav2_msgs

namespace nav2_msgs
{

namespace action
{

struct ComputeAndTrackRoute_SendGoal
{
  using Request = nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request;
  using Response = nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response;
};

}  // namespace action

}  // namespace nav2_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ComputeAndTrackRoute_GetResult_Request_
{
  using Type = ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator>;

  explicit ComputeAndTrackRoute_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit ComputeAndTrackRoute_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Request
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeAndTrackRoute_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeAndTrackRoute_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeAndTrackRoute_GetResult_Request_

// alias to use template instance with default allocator
using ComputeAndTrackRoute_GetResult_Request =
  nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace nav2_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "nav2_msgs/action/detail/compute_and_track_route__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ComputeAndTrackRoute_GetResult_Response_
{
  using Type = ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator>;

  explicit ComputeAndTrackRoute_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit ComputeAndTrackRoute_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const nav2_msgs::action::ComputeAndTrackRoute_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_GetResult_Response
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeAndTrackRoute_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeAndTrackRoute_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeAndTrackRoute_GetResult_Response_

// alias to use template instance with default allocator
using ComputeAndTrackRoute_GetResult_Response =
  nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace nav2_msgs

namespace nav2_msgs
{

namespace action
{

struct ComputeAndTrackRoute_GetResult
{
  using Request = nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request;
  using Response = nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response;
};

}  // namespace action

}  // namespace nav2_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "nav2_msgs/action/detail/compute_and_track_route__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ComputeAndTrackRoute_FeedbackMessage_
{
  using Type = ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator>;

  explicit ComputeAndTrackRoute_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit ComputeAndTrackRoute_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const nav2_msgs::action::ComputeAndTrackRoute_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__action__ComputeAndTrackRoute_FeedbackMessage
    std::shared_ptr<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeAndTrackRoute_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeAndTrackRoute_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeAndTrackRoute_FeedbackMessage_

// alias to use template instance with default allocator
using ComputeAndTrackRoute_FeedbackMessage =
  nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace nav2_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace nav2_msgs
{

namespace action
{

struct ComputeAndTrackRoute
{
  /// The goal message defined in the action definition.
  using Goal = nav2_msgs::action::ComputeAndTrackRoute_Goal;
  /// The result message defined in the action definition.
  using Result = nav2_msgs::action::ComputeAndTrackRoute_Result;
  /// The feedback message defined in the action definition.
  using Feedback = nav2_msgs::action::ComputeAndTrackRoute_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = nav2_msgs::action::ComputeAndTrackRoute_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = nav2_msgs::action::ComputeAndTrackRoute_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct ComputeAndTrackRoute ComputeAndTrackRoute;

}  // namespace action

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__ACTION__DETAIL__COMPUTE_AND_TRACK_ROUTE__STRUCT_HPP_
