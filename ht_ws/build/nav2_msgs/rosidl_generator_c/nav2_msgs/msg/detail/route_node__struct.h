// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav2_msgs:msg/RouteNode.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__ROUTE_NODE__STRUCT_H_
#define NAV2_MSGS__MSG__DETAIL__ROUTE_NODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/RouteNode in the package nav2_msgs.
typedef struct nav2_msgs__msg__RouteNode
{
  uint16_t nodeid;
  geometry_msgs__msg__Point position;
} nav2_msgs__msg__RouteNode;

// Struct for a sequence of nav2_msgs__msg__RouteNode.
typedef struct nav2_msgs__msg__RouteNode__Sequence
{
  nav2_msgs__msg__RouteNode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__msg__RouteNode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV2_MSGS__MSG__DETAIL__ROUTE_NODE__STRUCT_H_
