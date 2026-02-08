// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nav2_msgs:msg/EdgeCost.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__EDGE_COST__TRAITS_HPP_
#define NAV2_MSGS__MSG__DETAIL__EDGE_COST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nav2_msgs/msg/detail/edge_cost__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace nav2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EdgeCost & msg,
  std::ostream & out)
{
  out << "{";
  // member: edgeid
  {
    out << "edgeid: ";
    rosidl_generator_traits::value_to_yaml(msg.edgeid, out);
    out << ", ";
  }

  // member: cost
  {
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EdgeCost & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: edgeid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "edgeid: ";
    rosidl_generator_traits::value_to_yaml(msg.edgeid, out);
    out << "\n";
  }

  // member: cost
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EdgeCost & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::msg::EdgeCost & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::msg::EdgeCost & msg)
{
  return nav2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::msg::EdgeCost>()
{
  return "nav2_msgs::msg::EdgeCost";
}

template<>
inline const char * name<nav2_msgs::msg::EdgeCost>()
{
  return "nav2_msgs/msg/EdgeCost";
}

template<>
struct has_fixed_size<nav2_msgs::msg::EdgeCost>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<nav2_msgs::msg::EdgeCost>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<nav2_msgs::msg::EdgeCost>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NAV2_MSGS__MSG__DETAIL__EDGE_COST__TRAITS_HPP_
