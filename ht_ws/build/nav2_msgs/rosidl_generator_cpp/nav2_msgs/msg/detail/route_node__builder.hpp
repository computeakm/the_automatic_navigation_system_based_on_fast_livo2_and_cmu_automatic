// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_msgs:msg/RouteNode.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__ROUTE_NODE__BUILDER_HPP_
#define NAV2_MSGS__MSG__DETAIL__ROUTE_NODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav2_msgs/msg/detail/route_node__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav2_msgs
{

namespace msg
{

namespace builder
{

class Init_RouteNode_position
{
public:
  explicit Init_RouteNode_position(::nav2_msgs::msg::RouteNode & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::msg::RouteNode position(::nav2_msgs::msg::RouteNode::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::msg::RouteNode msg_;
};

class Init_RouteNode_nodeid
{
public:
  Init_RouteNode_nodeid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RouteNode_position nodeid(::nav2_msgs::msg::RouteNode::_nodeid_type arg)
  {
    msg_.nodeid = std::move(arg);
    return Init_RouteNode_position(msg_);
  }

private:
  ::nav2_msgs::msg::RouteNode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::msg::RouteNode>()
{
  return nav2_msgs::msg::builder::Init_RouteNode_nodeid();
}

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__MSG__DETAIL__ROUTE_NODE__BUILDER_HPP_
