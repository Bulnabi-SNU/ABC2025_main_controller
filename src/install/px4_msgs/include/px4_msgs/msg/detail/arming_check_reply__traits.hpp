// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/ArmingCheckReply.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ARMING_CHECK_REPLY__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ARMING_CHECK_REPLY__TRAITS_HPP_

#include "px4_msgs/msg/detail/arming_check_reply__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'events'
#include "px4_msgs/msg/detail/event__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<px4_msgs::msg::ArmingCheckReply>()
{
  return "px4_msgs::msg::ArmingCheckReply";
}

template<>
inline const char * name<px4_msgs::msg::ArmingCheckReply>()
{
  return "px4_msgs/msg/ArmingCheckReply";
}

template<>
struct has_fixed_size<px4_msgs::msg::ArmingCheckReply>
  : std::integral_constant<bool, has_fixed_size<px4_msgs::msg::Event>::value> {};

template<>
struct has_bounded_size<px4_msgs::msg::ArmingCheckReply>
  : std::integral_constant<bool, has_bounded_size<px4_msgs::msg::Event>::value> {};

template<>
struct is_message<px4_msgs::msg::ArmingCheckReply>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ARMING_CHECK_REPLY__TRAITS_HPP_
