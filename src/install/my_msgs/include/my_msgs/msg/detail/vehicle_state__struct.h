// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#ifndef MY_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
#define MY_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'state'
// Member 'substate'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/VehicleState in the package my_msgs.
typedef struct my_msgs__msg__VehicleState
{
  rosidl_runtime_c__String state;
  rosidl_runtime_c__String substate;
} my_msgs__msg__VehicleState;

// Struct for a sequence of my_msgs__msg__VehicleState.
typedef struct my_msgs__msg__VehicleState__Sequence
{
  my_msgs__msg__VehicleState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__msg__VehicleState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
