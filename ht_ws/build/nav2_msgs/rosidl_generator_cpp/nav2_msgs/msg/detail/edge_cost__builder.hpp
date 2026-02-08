// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_msgs:msg/EdgeCost.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__EDGE_COST__BUILDER_HPP_
#define NAV2_MSGS__MSG__DETAIL__EDGE_COST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav2_msgs/msg/detail/edge_cost__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav2_msgs
{

namespace msg
{

namespace builder
{

class Init_EdgeCost_cost
{
public:
  explicit Init_EdgeCost_cost(::nav2_msgs::msg::EdgeCost & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::msg::EdgeCost cost(::nav2_msgs::msg::EdgeCost::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::msg::EdgeCost msg_;
};

class Init_EdgeCost_edgeid
{
public:
  Init_EdgeCost_edgeid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EdgeCost_cost edgeid(::nav2_msgs::msg::EdgeCost::_edgeid_type arg)
  {
    msg_.edgeid = std::move(arg);
    return Init_EdgeCost_cost(msg_);
  }

private:
  ::nav2_msgs::msg::EdgeCost msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::msg::EdgeCost>()
{
  return nav2_msgs::msg::builder::Init_EdgeCost_edgeid();
}

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__MSG__DETAIL__EDGE_COST__BUILDER_HPP_
