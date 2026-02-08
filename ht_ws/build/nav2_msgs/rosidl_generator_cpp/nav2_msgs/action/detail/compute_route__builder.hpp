// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_msgs:action/ComputeRoute.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__ACTION__DETAIL__COMPUTE_ROUTE__BUILDER_HPP_
#define NAV2_MSGS__ACTION__DETAIL__COMPUTE_ROUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav2_msgs/action/detail/compute_route__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeRoute_Goal_use_poses
{
public:
  explicit Init_ComputeRoute_Goal_use_poses(::nav2_msgs::action::ComputeRoute_Goal & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeRoute_Goal use_poses(::nav2_msgs::action::ComputeRoute_Goal::_use_poses_type arg)
  {
    msg_.use_poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_Goal msg_;
};

class Init_ComputeRoute_Goal_use_start
{
public:
  explicit Init_ComputeRoute_Goal_use_start(::nav2_msgs::action::ComputeRoute_Goal & msg)
  : msg_(msg)
  {}
  Init_ComputeRoute_Goal_use_poses use_start(::nav2_msgs::action::ComputeRoute_Goal::_use_start_type arg)
  {
    msg_.use_start = std::move(arg);
    return Init_ComputeRoute_Goal_use_poses(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_Goal msg_;
};

class Init_ComputeRoute_Goal_goal
{
public:
  explicit Init_ComputeRoute_Goal_goal(::nav2_msgs::action::ComputeRoute_Goal & msg)
  : msg_(msg)
  {}
  Init_ComputeRoute_Goal_use_start goal(::nav2_msgs::action::ComputeRoute_Goal::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_ComputeRoute_Goal_use_start(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_Goal msg_;
};

class Init_ComputeRoute_Goal_goal_id
{
public:
  explicit Init_ComputeRoute_Goal_goal_id(::nav2_msgs::action::ComputeRoute_Goal & msg)
  : msg_(msg)
  {}
  Init_ComputeRoute_Goal_goal goal_id(::nav2_msgs::action::ComputeRoute_Goal::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ComputeRoute_Goal_goal(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_Goal msg_;
};

class Init_ComputeRoute_Goal_start
{
public:
  explicit Init_ComputeRoute_Goal_start(::nav2_msgs::action::ComputeRoute_Goal & msg)
  : msg_(msg)
  {}
  Init_ComputeRoute_Goal_goal_id start(::nav2_msgs::action::ComputeRoute_Goal::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_ComputeRoute_Goal_goal_id(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_Goal msg_;
};

class Init_ComputeRoute_Goal_start_id
{
public:
  Init_ComputeRoute_Goal_start_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeRoute_Goal_start start_id(::nav2_msgs::action::ComputeRoute_Goal::_start_id_type arg)
  {
    msg_.start_id = std::move(arg);
    return Init_ComputeRoute_Goal_start(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeRoute_Goal>()
{
  return nav2_msgs::action::builder::Init_ComputeRoute_Goal_start_id();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeRoute_Result_route
{
public:
  explicit Init_ComputeRoute_Result_route(::nav2_msgs::action::ComputeRoute_Result & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeRoute_Result route(::nav2_msgs::action::ComputeRoute_Result::_route_type arg)
  {
    msg_.route = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_Result msg_;
};

class Init_ComputeRoute_Result_path
{
public:
  explicit Init_ComputeRoute_Result_path(::nav2_msgs::action::ComputeRoute_Result & msg)
  : msg_(msg)
  {}
  Init_ComputeRoute_Result_route path(::nav2_msgs::action::ComputeRoute_Result::_path_type arg)
  {
    msg_.path = std::move(arg);
    return Init_ComputeRoute_Result_route(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_Result msg_;
};

class Init_ComputeRoute_Result_planning_time
{
public:
  Init_ComputeRoute_Result_planning_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeRoute_Result_path planning_time(::nav2_msgs::action::ComputeRoute_Result::_planning_time_type arg)
  {
    msg_.planning_time = std::move(arg);
    return Init_ComputeRoute_Result_path(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeRoute_Result>()
{
  return nav2_msgs::action::builder::Init_ComputeRoute_Result_planning_time();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeRoute_Feedback>()
{
  return ::nav2_msgs::action::ComputeRoute_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeRoute_SendGoal_Request_goal
{
public:
  explicit Init_ComputeRoute_SendGoal_Request_goal(::nav2_msgs::action::ComputeRoute_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeRoute_SendGoal_Request goal(::nav2_msgs::action::ComputeRoute_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_SendGoal_Request msg_;
};

class Init_ComputeRoute_SendGoal_Request_goal_id
{
public:
  Init_ComputeRoute_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeRoute_SendGoal_Request_goal goal_id(::nav2_msgs::action::ComputeRoute_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ComputeRoute_SendGoal_Request_goal(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeRoute_SendGoal_Request>()
{
  return nav2_msgs::action::builder::Init_ComputeRoute_SendGoal_Request_goal_id();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeRoute_SendGoal_Response_stamp
{
public:
  explicit Init_ComputeRoute_SendGoal_Response_stamp(::nav2_msgs::action::ComputeRoute_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeRoute_SendGoal_Response stamp(::nav2_msgs::action::ComputeRoute_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_SendGoal_Response msg_;
};

class Init_ComputeRoute_SendGoal_Response_accepted
{
public:
  Init_ComputeRoute_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeRoute_SendGoal_Response_stamp accepted(::nav2_msgs::action::ComputeRoute_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ComputeRoute_SendGoal_Response_stamp(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeRoute_SendGoal_Response>()
{
  return nav2_msgs::action::builder::Init_ComputeRoute_SendGoal_Response_accepted();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeRoute_GetResult_Request_goal_id
{
public:
  Init_ComputeRoute_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::nav2_msgs::action::ComputeRoute_GetResult_Request goal_id(::nav2_msgs::action::ComputeRoute_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeRoute_GetResult_Request>()
{
  return nav2_msgs::action::builder::Init_ComputeRoute_GetResult_Request_goal_id();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeRoute_GetResult_Response_result
{
public:
  explicit Init_ComputeRoute_GetResult_Response_result(::nav2_msgs::action::ComputeRoute_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeRoute_GetResult_Response result(::nav2_msgs::action::ComputeRoute_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_GetResult_Response msg_;
};

class Init_ComputeRoute_GetResult_Response_status
{
public:
  Init_ComputeRoute_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeRoute_GetResult_Response_result status(::nav2_msgs::action::ComputeRoute_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ComputeRoute_GetResult_Response_result(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeRoute_GetResult_Response>()
{
  return nav2_msgs::action::builder::Init_ComputeRoute_GetResult_Response_status();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace action
{

namespace builder
{

class Init_ComputeRoute_FeedbackMessage_feedback
{
public:
  explicit Init_ComputeRoute_FeedbackMessage_feedback(::nav2_msgs::action::ComputeRoute_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::action::ComputeRoute_FeedbackMessage feedback(::nav2_msgs::action::ComputeRoute_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_FeedbackMessage msg_;
};

class Init_ComputeRoute_FeedbackMessage_goal_id
{
public:
  Init_ComputeRoute_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeRoute_FeedbackMessage_feedback goal_id(::nav2_msgs::action::ComputeRoute_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ComputeRoute_FeedbackMessage_feedback(msg_);
  }

private:
  ::nav2_msgs::action::ComputeRoute_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::action::ComputeRoute_FeedbackMessage>()
{
  return nav2_msgs::action::builder::Init_ComputeRoute_FeedbackMessage_goal_id();
}

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__ACTION__DETAIL__COMPUTE_ROUTE__BUILDER_HPP_
