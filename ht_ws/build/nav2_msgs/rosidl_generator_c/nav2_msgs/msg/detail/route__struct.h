// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav2_msgs:msg/Route.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__ROUTE__STRUCT_H_
#define NAV2_MSGS__MSG__DETAIL__ROUTE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'nodes'
#include "nav2_msgs/msg/detail/route_node__struct.h"
// Member 'edges'
#include "nav2_msgs/msg/detail/route_edge__struct.h"

/// Struct defined in msg/Route in the package nav2_msgs.
typedef struct nav2_msgs__msg__Route
{
  std_msgs__msg__Header header;
  float route_cost;
  /// ordered set of nodes of the route
  nav2_msgs__msg__RouteNode__Sequence nodes;
  /// ordered set of edges that connect nodes
  nav2_msgs__msg__RouteEdge__Sequence edges;
} nav2_msgs__msg__Route;

// Struct for a sequence of nav2_msgs__msg__Route.
typedef struct nav2_msgs__msg__Route__Sequence
{
  nav2_msgs__msg__Route * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__msg__Route__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV2_MSGS__MSG__DETAIL__ROUTE__STRUCT_H_
