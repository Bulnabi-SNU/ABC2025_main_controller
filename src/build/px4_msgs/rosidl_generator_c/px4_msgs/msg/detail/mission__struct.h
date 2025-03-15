// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/Mission.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__MISSION__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__MISSION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Mission in the package px4_msgs.
typedef struct px4_msgs__msg__Mission
{
  uint64_t timestamp;
  uint8_t mission_dataman_id;
  uint8_t fence_dataman_id;
  uint8_t safepoint_dataman_id;
  uint16_t count;
  int32_t current_seq;
  int32_t land_start_index;
  int32_t land_index;
  uint32_t mission_id;
  uint32_t geofence_id;
  uint32_t safe_points_id;
} px4_msgs__msg__Mission;

// Struct for a sequence of px4_msgs__msg__Mission.
typedef struct px4_msgs__msg__Mission__Sequence
{
  px4_msgs__msg__Mission * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__Mission__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__MISSION__STRUCT_H_
