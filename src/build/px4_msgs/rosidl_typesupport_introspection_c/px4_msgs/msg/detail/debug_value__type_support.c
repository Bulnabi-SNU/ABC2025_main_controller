// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from px4_msgs:msg/DebugValue.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "px4_msgs/msg/detail/debug_value__rosidl_typesupport_introspection_c.h"
#include "px4_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "px4_msgs/msg/detail/debug_value__functions.h"
#include "px4_msgs/msg/detail/debug_value__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void DebugValue__rosidl_typesupport_introspection_c__DebugValue_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  px4_msgs__msg__DebugValue__init(message_memory);
}

void DebugValue__rosidl_typesupport_introspection_c__DebugValue_fini_function(void * message_memory)
{
  px4_msgs__msg__DebugValue__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember DebugValue__rosidl_typesupport_introspection_c__DebugValue_message_member_array[3] = {
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__DebugValue, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__DebugValue, ind),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__DebugValue, value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers DebugValue__rosidl_typesupport_introspection_c__DebugValue_message_members = {
  "px4_msgs__msg",  // message namespace
  "DebugValue",  // message name
  3,  // number of fields
  sizeof(px4_msgs__msg__DebugValue),
  DebugValue__rosidl_typesupport_introspection_c__DebugValue_message_member_array,  // message members
  DebugValue__rosidl_typesupport_introspection_c__DebugValue_init_function,  // function to initialize message memory (memory has to be allocated)
  DebugValue__rosidl_typesupport_introspection_c__DebugValue_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t DebugValue__rosidl_typesupport_introspection_c__DebugValue_message_type_support_handle = {
  0,
  &DebugValue__rosidl_typesupport_introspection_c__DebugValue_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_px4_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, px4_msgs, msg, DebugValue)() {
  if (!DebugValue__rosidl_typesupport_introspection_c__DebugValue_message_type_support_handle.typesupport_identifier) {
    DebugValue__rosidl_typesupport_introspection_c__DebugValue_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &DebugValue__rosidl_typesupport_introspection_c__DebugValue_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
