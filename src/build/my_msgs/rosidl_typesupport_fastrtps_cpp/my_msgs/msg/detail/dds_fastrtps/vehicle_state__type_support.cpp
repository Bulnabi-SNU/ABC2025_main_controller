// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from my_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice
#include "my_msgs/msg/detail/vehicle_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "my_msgs/msg/detail/vehicle_state__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace my_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_my_msgs
cdr_serialize(
  const my_msgs::msg::VehicleState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: state
  cdr << ros_message.state;
  // Member: substate
  cdr << ros_message.substate;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_my_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  my_msgs::msg::VehicleState & ros_message)
{
  // Member: state
  cdr >> ros_message.state;

  // Member: substate
  cdr >> ros_message.substate;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_my_msgs
get_serialized_size(
  const my_msgs::msg::VehicleState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: state
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.state.size() + 1);
  // Member: substate
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.substate.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_my_msgs
max_serialized_size_VehicleState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: state
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: substate
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static bool _VehicleState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const my_msgs::msg::VehicleState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _VehicleState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<my_msgs::msg::VehicleState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _VehicleState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const my_msgs::msg::VehicleState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _VehicleState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_VehicleState(full_bounded, 0);
}

static message_type_support_callbacks_t _VehicleState__callbacks = {
  "my_msgs::msg",
  "VehicleState",
  _VehicleState__cdr_serialize,
  _VehicleState__cdr_deserialize,
  _VehicleState__get_serialized_size,
  _VehicleState__max_serialized_size
};

static rosidl_message_type_support_t _VehicleState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_VehicleState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace my_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_my_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<my_msgs::msg::VehicleState>()
{
  return &my_msgs::msg::typesupport_fastrtps_cpp::_VehicleState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, my_msgs, msg, VehicleState)() {
  return &my_msgs::msg::typesupport_fastrtps_cpp::_VehicleState__handle;
}

#ifdef __cplusplus
}
#endif
