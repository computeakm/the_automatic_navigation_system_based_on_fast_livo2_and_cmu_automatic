// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav2_msgs:msg/EdgeCost.idl
// generated code does not contain a copyright notice

#ifndef NAV2_MSGS__MSG__DETAIL__EDGE_COST__STRUCT_H_
#define NAV2_MSGS__MSG__DETAIL__EDGE_COST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/EdgeCost in the package nav2_msgs.
/**
  * Edge cost to use with nav2_msgs/srv/DynamicEdges to adjust route edge costs
 */
typedef struct nav2_msgs__msg__EdgeCost
{
  uint16_t edgeid;
  float cost;
} nav2_msgs__msg__EdgeCost;

// Struct for a sequence of nav2_msgs__msg__EdgeCost.
typedef struct nav2_msgs__msg__EdgeCost__Sequence
{
  nav2_msgs__msg__EdgeCost * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__msg__EdgeCost__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV2_MSGS__MSG__DETAIL__EDGE_COST__STRUCT_H_
