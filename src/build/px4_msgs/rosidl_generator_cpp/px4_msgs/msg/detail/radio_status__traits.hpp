// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/RadioStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__RADIO_STATUS__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__RADIO_STATUS__TRAITS_HPP_

#include "px4_msgs/msg/detail/radio_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<px4_msgs::msg::RadioStatus>()
{
  return "px4_msgs::msg::RadioStatus";
}

template<>
inline const char * name<px4_msgs::msg::RadioStatus>()
{
  return "px4_msgs/msg/RadioStatus";
}

template<>
struct has_fixed_size<px4_msgs::msg::RadioStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::RadioStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::RadioStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__RADIO_STATUS__TRAITS_HPP_
