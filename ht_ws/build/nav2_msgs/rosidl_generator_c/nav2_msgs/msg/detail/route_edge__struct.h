// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav2_msgs:msg/RouteEdge.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__ROUTE_EDGE__STRUCT_H_
#define NAV2_MSGS__MSG__DETAIL__ROUTE_EDGE__STRUCT_H_

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
// Member 'end'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/RouteEdge in the package nav2_msgs.
typedef struct nav2_msgs__msg__RouteEdge
{
  uint16_t edgeid;
  geometry_msgs__msg__Point start;
  geometry_msgs__msg__Point end;
} nav2_msgs__msg__RouteEdge;

// Struct for a sequence of nav2_msgs__msg__RouteEdge.
typedef struct nav2_msgs__msg__RouteEdge__Sequence
{
  nav2_msgs__msg__RouteEdge * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__msg__RouteEdge__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV2_MSGS__MSG__DETAIL__ROUTE_EDGE__STRUCT_H_
