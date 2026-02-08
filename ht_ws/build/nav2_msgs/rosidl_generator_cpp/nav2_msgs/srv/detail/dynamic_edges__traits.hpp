// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nav2_msgs:srv/DynamicEdges.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__TRAITS_HPP_
#define NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nav2_msgs/srv/detail/dynamic_edges__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'adjust_edges'
#include "nav2_msgs/msg/detail/edge_cost__traits.hpp"

namespace nav2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const DynamicEdges_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: closed_edges
  {
    if (msg.closed_edges.size() == 0) {
      out << "closed_edges: []";
    } else {
      out << "closed_edges: [";
      size_t pending_items = msg.closed_edges.size();
      for (auto item : msg.closed_edges) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: opened_edges
  {
    if (msg.opened_edges.size() == 0) {
      out << "opened_edges: []";
    } else {
      out << "opened_edges: [";
      size_t pending_items = msg.opened_edges.size();
      for (auto item : msg.opened_edges) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: adjust_edges
  {
    if (msg.adjust_edges.size() == 0) {
      out << "adjust_edges: []";
    } else {
      out << "adjust_edges: [";
      size_t pending_items = msg.adjust_edges.size();
      for (auto item : msg.adjust_edges) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DynamicEdges_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: closed_edges
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.closed_edges.size() == 0) {
      out << "closed_edges: []\n";
    } else {
      out << "closed_edges:\n";
      for (auto item : msg.closed_edges) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: opened_edges
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.opened_edges.size() == 0) {
      out << "opened_edges: []\n";
    } else {
      out << "opened_edges:\n";
      for (auto item : msg.opened_edges) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: adjust_edges
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.adjust_edges.size() == 0) {
      out << "adjust_edges: []\n";
    } else {
      out << "adjust_edges:\n";
      for (auto item : msg.adjust_edges) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DynamicEdges_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::srv::DynamicEdges_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::srv::DynamicEdges_Request & msg)
{
  return nav2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::srv::DynamicEdges_Request>()
{
  return "nav2_msgs::srv::DynamicEdges_Request";
}

template<>
inline const char * name<nav2_msgs::srv::DynamicEdges_Request>()
{
  return "nav2_msgs/srv/DynamicEdges_Request";
}

template<>
struct has_fixed_size<nav2_msgs::srv::DynamicEdges_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nav2_msgs::srv::DynamicEdges_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<nav2_msgs::srv::DynamicEdges_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace nav2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const DynamicEdges_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DynamicEdges_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DynamicEdges_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::srv::DynamicEdges_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::srv::DynamicEdges_Response & msg)
{
  return nav2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::srv::DynamicEdges_Response>()
{
  return "nav2_msgs::srv::DynamicEdges_Response";
}

template<>
inline const char * name<nav2_msgs::srv::DynamicEdges_Response>()
{
  return "nav2_msgs/srv/DynamicEdges_Response";
}

template<>
struct has_fixed_size<nav2_msgs::srv::DynamicEdges_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<nav2_msgs::srv::DynamicEdges_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<nav2_msgs::srv::DynamicEdges_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nav2_msgs::srv::DynamicEdges>()
{
  return "nav2_msgs::srv::DynamicEdges";
}

template<>
inline const char * name<nav2_msgs::srv::DynamicEdges>()
{
  return "nav2_msgs/srv/DynamicEdges";
}

template<>
struct has_fixed_size<nav2_msgs::srv::DynamicEdges>
  : std::integral_constant<
    bool,
    has_fixed_size<nav2_msgs::srv::DynamicEdges_Request>::value &&
    has_fixed_size<nav2_msgs::srv::DynamicEdges_Response>::value
  >
{
};

template<>
struct has_bounded_size<nav2_msgs::srv::DynamicEdges>
  : std::integral_constant<
    bool,
    has_bounded_size<nav2_msgs::srv::DynamicEdges_Request>::value &&
    has_bounded_size<nav2_msgs::srv::DynamicEdges_Response>::value
  >
{
};

template<>
struct is_service<nav2_msgs::srv::DynamicEdges>
  : std::true_type
{
};

template<>
struct is_service_request<nav2_msgs::srv::DynamicEdges_Request>
  : std::true_type
{
};

template<>
struct is_service_response<nav2_msgs::srv::DynamicEdges_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__TRAITS_HPP_
