// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_msgs/msg/detail/vehicle_state__rosidl_typesupport_introspection_c.h"
#include "my_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_msgs/msg/detail/vehicle_state__functions.h"
#include "my_msgs/msg/detail/vehicle_state__struct.h"


// Include directives for member types
// Member `state`
// Member `substate`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void VehicleState__rosidl_typesupport_introspection_c__VehicleState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_msgs__msg__VehicleState__init(message_memory);
}

void VehicleState__rosidl_typesupport_introspection_c__VehicleState_fini_function(void * message_memory)
{
  my_msgs__msg__VehicleState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember VehicleState__rosidl_typesupport_introspection_c__VehicleState_message_member_array[2] = {
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_msgs__msg__VehicleState, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "substate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_msgs__msg__VehicleState, substate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers VehicleState__rosidl_typesupport_introspection_c__VehicleState_message_members = {
  "my_msgs__msg",  // message namespace
  "VehicleState",  // message name
  2,  // number of fields
  sizeof(my_msgs__msg__VehicleState),
  VehicleState__rosidl_typesupport_introspection_c__VehicleState_message_member_array,  // message members
  VehicleState__rosidl_typesupport_introspection_c__VehicleState_init_function,  // function to initialize message memory (memory has to be allocated)
  VehicleState__rosidl_typesupport_introspection_c__VehicleState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t VehicleState__rosidl_typesupport_introspection_c__VehicleState_message_type_support_handle = {
  0,
  &VehicleState__rosidl_typesupport_introspection_c__VehicleState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_msgs, msg, VehicleState)() {
  if (!VehicleState__rosidl_typesupport_introspection_c__VehicleState_message_type_support_handle.typesupport_identifier) {
    VehicleState__rosidl_typesupport_introspection_c__VehicleState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &VehicleState__rosidl_typesupport_introspection_c__VehicleState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
