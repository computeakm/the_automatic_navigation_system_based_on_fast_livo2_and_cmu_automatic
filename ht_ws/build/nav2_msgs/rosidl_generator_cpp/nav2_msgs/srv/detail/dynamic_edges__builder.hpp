// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_msgs:srv/DynamicEdges.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__BUILDER_HPP_
#define NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav2_msgs/srv/detail/dynamic_edges__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav2_msgs
{

namespace srv
{

namespace builder
{

class Init_DynamicEdges_Request_adjust_edges
{
public:
  explicit Init_DynamicEdges_Request_adjust_edges(::nav2_msgs::srv::DynamicEdges_Request & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::srv::DynamicEdges_Request adjust_edges(::nav2_msgs::srv::DynamicEdges_Request::_adjust_edges_type arg)
  {
    msg_.adjust_edges = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::srv::DynamicEdges_Request msg_;
};

class Init_DynamicEdges_Request_opened_edges
{
public:
  explicit Init_DynamicEdges_Request_opened_edges(::nav2_msgs::srv::DynamicEdges_Request & msg)
  : msg_(msg)
  {}
  Init_DynamicEdges_Request_adjust_edges opened_edges(::nav2_msgs::srv::DynamicEdges_Request::_opened_edges_type arg)
  {
    msg_.opened_edges = std::move(arg);
    return Init_DynamicEdges_Request_adjust_edges(msg_);
  }

private:
  ::nav2_msgs::srv::DynamicEdges_Request msg_;
};

class Init_DynamicEdges_Request_closed_edges
{
public:
  Init_DynamicEdges_Request_closed_edges()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DynamicEdges_Request_opened_edges closed_edges(::nav2_msgs::srv::DynamicEdges_Request::_closed_edges_type arg)
  {
    msg_.closed_edges = std::move(arg);
    return Init_DynamicEdges_Request_opened_edges(msg_);
  }

private:
  ::nav2_msgs::srv::DynamicEdges_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::srv::DynamicEdges_Request>()
{
  return nav2_msgs::srv::builder::Init_DynamicEdges_Request_closed_edges();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace srv
{

namespace builder
{

class Init_DynamicEdges_Response_success
{
public:
  Init_DynamicEdges_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::nav2_msgs::srv::DynamicEdges_Response success(::nav2_msgs::srv::DynamicEdges_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::srv::DynamicEdges_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::srv::DynamicEdges_Response>()
{
  return nav2_msgs::srv::builder::Init_DynamicEdges_Response_success();
}

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__BUILDER_HPP_
