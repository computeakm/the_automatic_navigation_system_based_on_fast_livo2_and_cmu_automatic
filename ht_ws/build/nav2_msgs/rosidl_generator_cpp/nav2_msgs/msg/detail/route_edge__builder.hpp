// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_msgs:msg/RouteEdge.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__ROUTE_EDGE__BUILDER_HPP_
#define NAV2_MSGS__MSG__DETAIL__ROUTE_EDGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav2_msgs/msg/detail/route_edge__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav2_msgs
{

namespace msg
{

namespace builder
{

class Init_RouteEdge_end
{
public:
  explicit Init_RouteEdge_end(::nav2_msgs::msg::RouteEdge & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::msg::RouteEdge end(::nav2_msgs::msg::RouteEdge::_end_type arg)
  {
    msg_.end = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::msg::RouteEdge msg_;
};

class Init_RouteEdge_start
{
public:
  explicit Init_RouteEdge_start(::nav2_msgs::msg::RouteEdge & msg)
  : msg_(msg)
  {}
  Init_RouteEdge_end start(::nav2_msgs::msg::RouteEdge::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_RouteEdge_end(msg_);
  }

private:
  ::nav2_msgs::msg::RouteEdge msg_;
};

class Init_RouteEdge_edgeid
{
public:
  Init_RouteEdge_edgeid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RouteEdge_start edgeid(::nav2_msgs::msg::RouteEdge::_edgeid_type arg)
  {
    msg_.edgeid = std::move(arg);
    return Init_RouteEdge_start(msg_);
  }

private:
  ::nav2_msgs::msg::RouteEdge msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::msg::RouteEdge>()
{
  return nav2_msgs::msg::builder::Init_RouteEdge_edgeid();
}

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__MSG__DETAIL__ROUTE_EDGE__BUILDER_HPP_
