// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_msgs:msg/DepthActivated.idl
// generated code does not contain a copyright notice

#ifndef MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__STRUCT_H_
#define MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/DepthActivated in the package my_msgs.
typedef struct my_msgs__msg__DepthActivated
{
  bool depth_activated;
} my_msgs__msg__DepthActivated;

// Struct for a sequence of my_msgs__msg__DepthActivated.
typedef struct my_msgs__msg__DepthActivated__Sequence
{
  my_msgs__msg__DepthActivated * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__msg__DepthActivated__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__STRUCT_H_
