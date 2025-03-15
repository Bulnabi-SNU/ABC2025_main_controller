// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_msgs:msg/DepthActivated.idl
// generated code does not contain a copyright notice

#ifndef MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__TRAITS_HPP_
#define MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__TRAITS_HPP_

#include "my_msgs/msg/detail/depth_activated__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<my_msgs::msg::DepthActivated>()
{
  return "my_msgs::msg::DepthActivated";
}

template<>
inline const char * name<my_msgs::msg::DepthActivated>()
{
  return "my_msgs/msg/DepthActivated";
}

template<>
struct has_fixed_size<my_msgs::msg::DepthActivated>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_msgs::msg::DepthActivated>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_msgs::msg::DepthActivated>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__TRAITS_HPP_
