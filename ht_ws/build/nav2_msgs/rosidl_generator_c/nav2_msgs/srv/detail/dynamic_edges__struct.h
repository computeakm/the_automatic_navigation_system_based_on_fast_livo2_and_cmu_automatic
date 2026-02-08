// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav2_msgs:srv/DynamicEdges.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__STRUCT_H_
#define NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'closed_edges'
// Member 'opened_edges'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'adjust_edges'
#include "nav2_msgs/msg/detail/edge_cost__struct.h"

/// Struct defined in srv/DynamicEdges in the package nav2_msgs.
typedef struct nav2_msgs__srv__DynamicEdges_Request
{
  rosidl_runtime_c__uint16__Sequence closed_edges;
  rosidl_runtime_c__uint16__Sequence opened_edges;
  nav2_msgs__msg__EdgeCost__Sequence adjust_edges;
} nav2_msgs__srv__DynamicEdges_Request;

// Struct for a sequence of nav2_msgs__srv__DynamicEdges_Request.
typedef struct nav2_msgs__srv__DynamicEdges_Request__Sequence
{
  nav2_msgs__srv__DynamicEdges_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__srv__DynamicEdges_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/DynamicEdges in the package nav2_msgs.
typedef struct nav2_msgs__srv__DynamicEdges_Response
{
  bool success;
} nav2_msgs__srv__DynamicEdges_Response;

// Struct for a sequence of nav2_msgs__srv__DynamicEdges_Response.
typedef struct nav2_msgs__srv__DynamicEdges_Response__Sequence
{
  nav2_msgs__srv__DynamicEdges_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__srv__DynamicEdges_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV2_MSGS__SRV__DETAIL__DYNAMIC_EDGES__STRUCT_H_
