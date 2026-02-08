// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nav2_msgs:action/ComputeAndTrackRoute.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__ACTION__DETAIL__COMPUTE_AND_TRACK_ROUTE__TRAITS_HPP_
#define NAV2_MSGS__ACTION__DETAIL__COMPUTE_AND_TRACK_ROUTE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nav2_msgs/action/detail/compute_and_track_route__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'start'
// Member 'goal'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace nav2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ComputeAndTrackRoute_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: start_id
  {
    out << "start_id: ";
    rosidl_generator_traits::value_to_yaml(msg.start_id, out);
    out << ", ";
  }

  // member: start
  {
    out << "start: ";
    to_flow_style_yaml(msg.start, out);
    out << ", ";
  }

  // member: goal_id
  {
    out << "goal_id: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
    out << ", ";
  }

  // member: use_start
  {
    out << "use_start: ";
    rosidl_generator_traits::value_to_yaml(msg.use_start, out);
    out << ", ";
  }

  // member: use_poses
  {
    out << "use_poses: ";
    rosidl_generator_traits::value_to_yaml(msg.use_poses, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeAndTrackRoute_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_id: ";
    rosidl_generator_traits::value_to_yaml(msg.start_id, out);
    out << "\n";
  }

  // member: start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start:\n";
    to_block_style_yaml(msg.start, out, indentation + 2);
  }

  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_id, out);
    out << "\n";
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }

  // member: use_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_start: ";
    rosidl_generator_traits::value_to_yaml(msg.use_start, out);
    out << "\n";
  }

  // member: use_poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_poses: ";
    rosidl_generator_traits::value_to_yaml(msg.use_poses, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeAndTrackRoute_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::action::ComputeAndTrackRoute_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::action::ComputeAndTrackRoute_Goal & msg)
{
  return nav2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_Goal>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_Goal";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_Goal>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_Goal";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_Goal>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_Goal>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<nav2_msgs::action::ComputeAndTrackRoute_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'execution_duration'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace nav2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ComputeAndTrackRoute_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: execution_duration
  {
    out << "execution_duration: ";
    to_flow_style_yaml(msg.execution_duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeAndTrackRoute_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: execution_duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "execution_duration:\n";
    to_block_style_yaml(msg.execution_duration, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeAndTrackRoute_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::action::ComputeAndTrackRoute_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::action::ComputeAndTrackRoute_Result & msg)
{
  return nav2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_Result>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_Result";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_Result>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_Result";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_Result>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Duration>::value> {};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_Result>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Duration>::value> {};

template<>
struct is_message<nav2_msgs::action::ComputeAndTrackRoute_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'route'
#include "nav2_msgs/msg/detail/route__traits.hpp"
// Member 'path'
#include "nav_msgs/msg/detail/path__traits.hpp"

namespace nav2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ComputeAndTrackRoute_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: last_node_id
  {
    out << "last_node_id: ";
    rosidl_generator_traits::value_to_yaml(msg.last_node_id, out);
    out << ", ";
  }

  // member: next_node_id
  {
    out << "next_node_id: ";
    rosidl_generator_traits::value_to_yaml(msg.next_node_id, out);
    out << ", ";
  }

  // member: current_edge_id
  {
    out << "current_edge_id: ";
    rosidl_generator_traits::value_to_yaml(msg.current_edge_id, out);
    out << ", ";
  }

  // member: route
  {
    out << "route: ";
    to_flow_style_yaml(msg.route, out);
    out << ", ";
  }

  // member: path
  {
    out << "path: ";
    to_flow_style_yaml(msg.path, out);
    out << ", ";
  }

  // member: operations_triggered
  {
    if (msg.operations_triggered.size() == 0) {
      out << "operations_triggered: []";
    } else {
      out << "operations_triggered: [";
      size_t pending_items = msg.operations_triggered.size();
      for (auto item : msg.operations_triggered) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: rerouted
  {
    out << "rerouted: ";
    rosidl_generator_traits::value_to_yaml(msg.rerouted, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeAndTrackRoute_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: last_node_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_node_id: ";
    rosidl_generator_traits::value_to_yaml(msg.last_node_id, out);
    out << "\n";
  }

  // member: next_node_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "next_node_id: ";
    rosidl_generator_traits::value_to_yaml(msg.next_node_id, out);
    out << "\n";
  }

  // member: current_edge_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_edge_id: ";
    rosidl_generator_traits::value_to_yaml(msg.current_edge_id, out);
    out << "\n";
  }

  // member: route
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "route:\n";
    to_block_style_yaml(msg.route, out, indentation + 2);
  }

  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path:\n";
    to_block_style_yaml(msg.path, out, indentation + 2);
  }

  // member: operations_triggered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.operations_triggered.size() == 0) {
      out << "operations_triggered: []\n";
    } else {
      out << "operations_triggered:\n";
      for (auto item : msg.operations_triggered) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: rerouted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rerouted: ";
    rosidl_generator_traits::value_to_yaml(msg.rerouted, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeAndTrackRoute_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::action::ComputeAndTrackRoute_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::action::ComputeAndTrackRoute_Feedback & msg)
{
  return nav2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_Feedback>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_Feedback";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_Feedback>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_Feedback";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<nav2_msgs::action::ComputeAndTrackRoute_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "nav2_msgs/action/detail/compute_and_track_route__traits.hpp"

namespace nav2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ComputeAndTrackRoute_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeAndTrackRoute_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeAndTrackRoute_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request & msg)
{
  return nav2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_SendGoal_Request";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace nav2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ComputeAndTrackRoute_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeAndTrackRoute_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeAndTrackRoute_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response & msg)
{
  return nav2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_SendGoal_Response";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_SendGoal>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_SendGoal";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_SendGoal>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_SendGoal";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request>::value &&
    has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request>::value &&
    has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<nav2_msgs::action::ComputeAndTrackRoute_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace nav2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ComputeAndTrackRoute_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeAndTrackRoute_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeAndTrackRoute_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request & msg)
{
  return nav2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_GetResult_Request";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "nav2_msgs/action/detail/compute_and_track_route__traits.hpp"

namespace nav2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ComputeAndTrackRoute_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeAndTrackRoute_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeAndTrackRoute_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response & msg)
{
  return nav2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_GetResult_Response";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_Result>::value> {};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_Result>::value> {};

template<>
struct is_message<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_GetResult>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_GetResult";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_GetResult>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_GetResult";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request>::value &&
    has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request>::value &&
    has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response>::value
  >
{
};

template<>
struct is_service<nav2_msgs::action::ComputeAndTrackRoute_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "nav2_msgs/action/detail/compute_and_track_route__traits.hpp"

namespace nav2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ComputeAndTrackRoute_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeAndTrackRoute_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeAndTrackRoute_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage & msg)
{
  return nav2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage>()
{
  return "nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage";
}

template<>
inline const char * name<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage>()
{
  return "nav2_msgs/action/ComputeAndTrackRoute_FeedbackMessage";
}

template<>
struct has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<nav2_msgs::action::ComputeAndTrackRoute_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<nav2_msgs::action::ComputeAndTrackRoute_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<nav2_msgs::action::ComputeAndTrackRoute>
  : std::true_type
{
};

template<>
struct is_action_goal<nav2_msgs::action::ComputeAndTrackRoute_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<nav2_msgs::action::ComputeAndTrackRoute_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<nav2_msgs::action::ComputeAndTrackRoute_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // NAV2_MSGS__ACTION__DETAIL__COMPUTE_AND_TRACK_ROUTE__TRAITS_HPP_
