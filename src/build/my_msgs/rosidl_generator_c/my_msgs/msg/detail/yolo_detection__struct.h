// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_msgs:msg/YoloDetection.idl
// generated code does not contain a copyright notice

#ifndef MY_MSGS__MSG__DETAIL__YOLO_DETECTION__STRUCT_H_
#define MY_MSGS__MSG__DETAIL__YOLO_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'label'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/YoloDetection in the package my_msgs.
typedef struct my_msgs__msg__YoloDetection
{
  rosidl_runtime_c__String label;
  float screen_width;
  float screen_height;
  float xmax;
  float ymax;
  float xmin;
  float ymin;
} my_msgs__msg__YoloDetection;

// Struct for a sequence of my_msgs__msg__YoloDetection.
typedef struct my_msgs__msg__YoloDetection__Sequence
{
  my_msgs__msg__YoloDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__msg__YoloDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_MSGS__MSG__DETAIL__YOLO_DETECTION__STRUCT_H_
