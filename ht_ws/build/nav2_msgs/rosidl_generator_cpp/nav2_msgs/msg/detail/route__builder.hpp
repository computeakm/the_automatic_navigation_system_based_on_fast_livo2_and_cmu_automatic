// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_msgs:msg/Route.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__ROUTE__BUILDER_HPP_
#define NAV2_MSGS__MSG__DETAIL__ROUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav2_msgs/msg/detail/route__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav2_msgs
{

namespace msg
{

namespace builder
{

class Init_Route_edges
{
public:
  explicit Init_Route_edges(::nav2_msgs::msg::Route & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::msg::Route edges(::nav2_msgs::msg::Route::_edges_type arg)
  {
    msg_.edges = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::msg::Route msg_;
};

class Init_Route_nodes
{
public:
  explicit Init_Route_nodes(::nav2_msgs::msg::Route & msg)
  : msg_(msg)
  {}
  Init_Route_edges nodes(::nav2_msgs::msg::Route::_nodes_type arg)
  {
    msg_.nodes = std::move(arg);
    return Init_Route_edges(msg_);
  }

private:
  ::nav2_msgs::msg::Route msg_;
};

class Init_Route_route_cost
{
public:
  explicit Init_Route_route_cost(::nav2_msgs::msg::Route & msg)
  : msg_(msg)
  {}
  Init_Route_nodes route_cost(::nav2_msgs::msg::Route::_route_cost_type arg)
  {
    msg_.route_cost = std::move(arg);
    return Init_Route_nodes(msg_);
  }

private:
  ::nav2_msgs::msg::Route msg_;
};

class Init_Route_header
{
public:
  Init_Route_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Route_route_cost header(::nav2_msgs::msg::Route::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Route_route_cost(msg_);
  }

private:
  ::nav2_msgs::msg::Route msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::msg::Route>()
{
  return nav2_msgs::msg::builder::Init_Route_header();
}

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__MSG__DETAIL__ROUTE__BUILDER_HPP_
