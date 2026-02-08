// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_msgs:action/ComputeAndTrackRoute.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__ACTION__DETAIL__COMPUTE_AND_TRACK_ROUTE__BUILDER_HPP_
#define NAV2_MSGS__ACTION__DETAIL__COMPUTE_AND_TRACK_ROUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav2_msgs/action/detail/compute_and_track_route__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeAndTrackRoute_Goal_use_poses
{
public:
  explicit Init_ComputeAndTrackRoute_Goal_use_poses(::nav2_msgs::action::ComputeAndTrackRoute_Goal & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeAndTrackRoute_Goal use_poses(::nav2_msgs::action::ComputeAndTrackRoute_Goal::_use_poses_type arg)
  {
    msg_.use_poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Goal msg_;
};

class Init_ComputeAndTrackRoute_Goal_use_start
{
public:
  explicit Init_ComputeAndTrackRoute_Goal_use_start(::nav2_msgs::action::ComputeAndTrackRoute_Goal & msg)
  : msg_(msg)
  {}
  Init_ComputeAndTrackRoute_Goal_use_poses use_start(::nav2_msgs::action::ComputeAndTrackRoute_Goal::_use_start_type arg)
  {
    msg_.use_start = std::move(arg);
    return Init_ComputeAndTrackRoute_Goal_use_poses(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Goal msg_;
};

class Init_ComputeAndTrackRoute_Goal_goal
{
public:
  explicit Init_ComputeAndTrackRoute_Goal_goal(::nav2_msgs::action::ComputeAndTrackRoute_Goal & msg)
  : msg_(msg)
  {}
  Init_ComputeAndTrackRoute_Goal_use_start goal(::nav2_msgs::action::ComputeAndTrackRoute_Goal::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_ComputeAndTrackRoute_Goal_use_start(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Goal msg_;
};

class Init_ComputeAndTrackRoute_Goal_goal_id
{
public:
  explicit Init_ComputeAndTrackRoute_Goal_goal_id(::nav2_msgs::action::ComputeAndTrackRoute_Goal & msg)
  : msg_(msg)
  {}
  Init_ComputeAndTrackRoute_Goal_goal goal_id(::nav2_msgs::action::ComputeAndTrackRoute_Goal::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ComputeAndTrackRoute_Goal_goal(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Goal msg_;
};

class Init_ComputeAndTrackRoute_Goal_start
{
public:
  explicit Init_ComputeAndTrackRoute_Goal_start(::nav2_msgs::action::ComputeAndTrackRoute_Goal & msg)
  : msg_(msg)
  {}
  Init_ComputeAndTrackRoute_Goal_goal_id start(::nav2_msgs::action::ComputeAndTrackRoute_Goal::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_ComputeAndTrackRoute_Goal_goal_id(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Goal msg_;
};

class Init_ComputeAndTrackRoute_Goal_start_id
{
public:
  Init_ComputeAndTrackRoute_Goal_start_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeAndTrackRoute_Goal_start start_id(::nav2_msgs::action::ComputeAndTrackRoute_Goal::_start_id_type arg)
  {
    msg_.start_id = std::move(arg);
    return Init_ComputeAndTrackRoute_Goal_start(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeAndTrackRoute_Goal>()
{
  return nav2_msgs::action::builder::Init_ComputeAndTrackRoute_Goal_start_id();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeAndTrackRoute_Result_execution_duration
{
public:
  Init_ComputeAndTrackRoute_Result_execution_duration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::nav2_msgs::action::ComputeAndTrackRoute_Result execution_duration(::nav2_msgs::action::ComputeAndTrackRoute_Result::_execution_duration_type arg)
  {
    msg_.execution_duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeAndTrackRoute_Result>()
{
  return nav2_msgs::action::builder::Init_ComputeAndTrackRoute_Result_execution_duration();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeAndTrackRoute_Feedback_rerouted
{
public:
  explicit Init_ComputeAndTrackRoute_Feedback_rerouted(::nav2_msgs::action::ComputeAndTrackRoute_Feedback & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeAndTrackRoute_Feedback rerouted(::nav2_msgs::action::ComputeAndTrackRoute_Feedback::_rerouted_type arg)
  {
    msg_.rerouted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Feedback msg_;
};

class Init_ComputeAndTrackRoute_Feedback_operations_triggered
{
public:
  explicit Init_ComputeAndTrackRoute_Feedback_operations_triggered(::nav2_msgs::action::ComputeAndTrackRoute_Feedback & msg)
  : msg_(msg)
  {}
  Init_ComputeAndTrackRoute_Feedback_rerouted operations_triggered(::nav2_msgs::action::ComputeAndTrackRoute_Feedback::_operations_triggered_type arg)
  {
    msg_.operations_triggered = std::move(arg);
    return Init_ComputeAndTrackRoute_Feedback_rerouted(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Feedback msg_;
};

class Init_ComputeAndTrackRoute_Feedback_path
{
public:
  explicit Init_ComputeAndTrackRoute_Feedback_path(::nav2_msgs::action::ComputeAndTrackRoute_Feedback & msg)
  : msg_(msg)
  {}
  Init_ComputeAndTrackRoute_Feedback_operations_triggered path(::nav2_msgs::action::ComputeAndTrackRoute_Feedback::_path_type arg)
  {
    msg_.path = std::move(arg);
    return Init_ComputeAndTrackRoute_Feedback_operations_triggered(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Feedback msg_;
};

class Init_ComputeAndTrackRoute_Feedback_route
{
public:
  explicit Init_ComputeAndTrackRoute_Feedback_route(::nav2_msgs::action::ComputeAndTrackRoute_Feedback & msg)
  : msg_(msg)
  {}
  Init_ComputeAndTrackRoute_Feedback_path route(::nav2_msgs::action::ComputeAndTrackRoute_Feedback::_route_type arg)
  {
    msg_.route = std::move(arg);
    return Init_ComputeAndTrackRoute_Feedback_path(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Feedback msg_;
};

class Init_ComputeAndTrackRoute_Feedback_current_edge_id
{
public:
  explicit Init_ComputeAndTrackRoute_Feedback_current_edge_id(::nav2_msgs::action::ComputeAndTrackRoute_Feedback & msg)
  : msg_(msg)
  {}
  Init_ComputeAndTrackRoute_Feedback_route current_edge_id(::nav2_msgs::action::ComputeAndTrackRoute_Feedback::_current_edge_id_type arg)
  {
    msg_.current_edge_id = std::move(arg);
    return Init_ComputeAndTrackRoute_Feedback_route(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Feedback msg_;
};

class Init_ComputeAndTrackRoute_Feedback_next_node_id
{
public:
  explicit Init_ComputeAndTrackRoute_Feedback_next_node_id(::nav2_msgs::action::ComputeAndTrackRoute_Feedback & msg)
  : msg_(msg)
  {}
  Init_ComputeAndTrackRoute_Feedback_current_edge_id next_node_id(::nav2_msgs::action::ComputeAndTrackRoute_Feedback::_next_node_id_type arg)
  {
    msg_.next_node_id = std::move(arg);
    return Init_ComputeAndTrackRoute_Feedback_current_edge_id(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Feedback msg_;
};

class Init_ComputeAndTrackRoute_Feedback_last_node_id
{
public:
  Init_ComputeAndTrackRoute_Feedback_last_node_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeAndTrackRoute_Feedback_next_node_id last_node_id(::nav2_msgs::action::ComputeAndTrackRoute_Feedback::_last_node_id_type arg)
  {
    msg_.last_node_id = std::move(arg);
    return Init_ComputeAndTrackRoute_Feedback_next_node_id(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeAndTrackRoute_Feedback>()
{
  return nav2_msgs::action::builder::Init_ComputeAndTrackRoute_Feedback_last_node_id();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeAndTrackRoute_SendGoal_Request_goal
{
public:
  explicit Init_ComputeAndTrackRoute_SendGoal_Request_goal(::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request goal(::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request msg_;
};

class Init_ComputeAndTrackRoute_SendGoal_Request_goal_id
{
public:
  Init_ComputeAndTrackRoute_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeAndTrackRoute_SendGoal_Request_goal goal_id(::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ComputeAndTrackRoute_SendGoal_Request_goal(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Request>()
{
  return nav2_msgs::action::builder::Init_ComputeAndTrackRoute_SendGoal_Request_goal_id();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeAndTrackRoute_SendGoal_Response_stamp
{
public:
  explicit Init_ComputeAndTrackRoute_SendGoal_Response_stamp(::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response stamp(::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response msg_;
};

class Init_ComputeAndTrackRoute_SendGoal_Response_accepted
{
public:
  Init_ComputeAndTrackRoute_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeAndTrackRoute_SendGoal_Response_stamp accepted(::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ComputeAndTrackRoute_SendGoal_Response_stamp(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeAndTrackRoute_SendGoal_Response>()
{
  return nav2_msgs::action::builder::Init_ComputeAndTrackRoute_SendGoal_Response_accepted();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeAndTrackRoute_GetResult_Request_goal_id
{
public:
  Init_ComputeAndTrackRoute_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request goal_id(::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Request>()
{
  return nav2_msgs::action::builder::Init_ComputeAndTrackRoute_GetResult_Request_goal_id();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeAndTrackRoute_GetResult_Response_result
{
public:
  explicit Init_ComputeAndTrackRoute_GetResult_Response_result(::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response result(::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response msg_;
};

class Init_ComputeAndTrackRoute_GetResult_Response_status
{
public:
  Init_ComputeAndTrackRoute_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeAndTrackRoute_GetResult_Response_result status(::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ComputeAndTrackRoute_GetResult_Response_result(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeAndTrackRoute_GetResult_Response>()
{
  return nav2_msgs::action::builder::Init_ComputeAndTrackRoute_GetResult_Response_status();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeAndTrackRoute_FeedbackMessage_feedback
{
public:
  explicit Init_ComputeAndTrackRoute_FeedbackMessage_feedback(::nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage feedback(::nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage msg_;
};

class Init_ComputeAndTrackRoute_FeedbackMessage_goal_id
{
public:
  Init_ComputeAndTrackRoute_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeAndTrackRoute_FeedbackMessage_feedback goal_id(::nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ComputeAndTrackRoute_FeedbackMessage_feedback(msg_);
  }

private:
  ::nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeAndTrackRoute_FeedbackMessage>()
{
  return nav2_msgs::action::builder::Init_ComputeAndTrackRoute_FeedbackMessage_goal_id();
}

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__ACTION__DETAIL__COMPUTE_AND_TRACK_ROUTE__BUILDER_HPP_
