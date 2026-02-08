// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav2_msgs:action/ComputeRoute.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__ACTION__DETAIL__COMPUTE_ROUTE__STRUCT_H_
#define NAV2_MSGS__ACTION__DETAIL__COMPUTE_ROUTE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start'
// Member 'goal'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/ComputeRoute in the package nav2_msgs.
typedef struct nav2_msgs__action__ComputeRoute_Goal
{
  uint16_t start_id;
  geometry_msgs__msg__PoseStamped start;
  uint16_t goal_id;
  geometry_msgs__msg__PoseStamped goal;
  /// Whether to use the start field or find the start pose in TF
  bool use_start;
  /// Whether to use the poses or the IDs fields for request
  bool use_poses;
} nav2_msgs__action__ComputeRoute_Goal;

// Struct for a sequence of nav2_msgs__action__ComputeRoute_Goal.
typedef struct nav2_msgs__action__ComputeRoute_Goal__Sequence
{
  nav2_msgs__action__ComputeRoute_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__action__ComputeRoute_Goal__Sequence;


// Constants defined in the message

/// Constant 'NONE'.
/**
  * Error codes
 */
enum
{
  nav2_msgs__action__ComputeRoute_Result__NONE = 0
};

/// Constant 'UNKNOWN'.
enum
{
  nav2_msgs__action__ComputeRoute_Result__UNKNOWN = 400
};

/// Constant 'TF_ERROR'.
enum
{
  nav2_msgs__action__ComputeRoute_Result__TF_ERROR = 401
};

/// Constant 'NO_VALID_GRAPH'.
enum
{
  nav2_msgs__action__ComputeRoute_Result__NO_VALID_GRAPH = 402
};

/// Constant 'INDETERMINANT_NODES_ON_GRAPH'.
enum
{
  nav2_msgs__action__ComputeRoute_Result__INDETERMINANT_NODES_ON_GRAPH = 403
};

/// Constant 'TIMEOUT'.
enum
{
  nav2_msgs__action__ComputeRoute_Result__TIMEOUT = 404
};

/// Constant 'NO_VALID_ROUTE'.
enum
{
  nav2_msgs__action__ComputeRoute_Result__NO_VALID_ROUTE = 405
};

/// Constant 'INVALID_EDGE_SCORER_USE'.
enum
{
  nav2_msgs__action__ComputeRoute_Result__INVALID_EDGE_SCORER_USE = 407
};

// Include directives for member types
// Member 'planning_time'
#include "builtin_interfaces/msg/detail/duration__struct.h"
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"
// Member 'route'
#include "nav2_msgs/msg/detail/route__struct.h"

/// Struct defined in action/ComputeRoute in the package nav2_msgs.
typedef struct nav2_msgs__action__ComputeRoute_Result
{
  builtin_interfaces__msg__Duration planning_time;
  nav_msgs__msg__Path path;
  nav2_msgs__msg__Route route;
} nav2_msgs__action__ComputeRoute_Result;

// Struct for a sequence of nav2_msgs__action__ComputeRoute_Result.
typedef struct nav2_msgs__action__ComputeRoute_Result__Sequence
{
  nav2_msgs__action__ComputeRoute_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__action__ComputeRoute_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/ComputeRoute in the package nav2_msgs.
typedef struct nav2_msgs__action__ComputeRoute_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} nav2_msgs__action__ComputeRoute_Feedback;

// Struct for a sequence of nav2_msgs__action__ComputeRoute_Feedback.
typedef struct nav2_msgs__action__ComputeRoute_Feedback__Sequence
{
  nav2_msgs__action__ComputeRoute_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__action__ComputeRoute_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "nav2_msgs/action/detail/compute_route__struct.h"

/// Struct defined in action/ComputeRoute in the package nav2_msgs.
typedef struct nav2_msgs__action__ComputeRoute_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  nav2_msgs__action__ComputeRoute_Goal goal;
} nav2_msgs__action__ComputeRoute_SendGoal_Request;

// Struct for a sequence of nav2_msgs__action__ComputeRoute_SendGoal_Request.
typedef struct nav2_msgs__action__ComputeRoute_SendGoal_Request__Sequence
{
  nav2_msgs__action__ComputeRoute_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__action__ComputeRoute_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ComputeRoute in the package nav2_msgs.
typedef struct nav2_msgs__action__ComputeRoute_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} nav2_msgs__action__ComputeRoute_SendGoal_Response;

// Struct for a sequence of nav2_msgs__action__ComputeRoute_SendGoal_Response.
typedef struct nav2_msgs__action__ComputeRoute_SendGoal_Response__Sequence
{
  nav2_msgs__action__ComputeRoute_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__action__ComputeRoute_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ComputeRoute in the package nav2_msgs.
typedef struct nav2_msgs__action__ComputeRoute_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} nav2_msgs__action__ComputeRoute_GetResult_Request;

// Struct for a sequence of nav2_msgs__action__ComputeRoute_GetResult_Request.
typedef struct nav2_msgs__action__ComputeRoute_GetResult_Request__Sequence
{
  nav2_msgs__action__ComputeRoute_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__action__ComputeRoute_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "nav2_msgs/action/detail/compute_route__struct.h"

/// Struct defined in action/ComputeRoute in the package nav2_msgs.
typedef struct nav2_msgs__action__ComputeRoute_GetResult_Response
{
  int8_t status;
  nav2_msgs__action__ComputeRoute_Result result;
} nav2_msgs__action__ComputeRoute_GetResult_Response;

// Struct for a sequence of nav2_msgs__action__ComputeRoute_GetResult_Response.
typedef struct nav2_msgs__action__ComputeRoute_GetResult_Response__Sequence
{
  nav2_msgs__action__ComputeRoute_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__action__ComputeRoute_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "nav2_msgs/action/detail/compute_route__struct.h"

/// Struct defined in action/ComputeRoute in the package nav2_msgs.
typedef struct nav2_msgs__action__ComputeRoute_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  nav2_msgs__action__ComputeRoute_Feedback feedback;
} nav2_msgs__action__ComputeRoute_FeedbackMessage;

// Struct for a sequence of nav2_msgs__action__ComputeRoute_FeedbackMessage.
typedef struct nav2_msgs__action__ComputeRoute_FeedbackMessage__Sequence
{
  nav2_msgs__action__ComputeRoute_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__action__ComputeRoute_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV2_MSGS__ACTION__DETAIL__COMPUTE_ROUTE__STRUCT_H_
