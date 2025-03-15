// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_msgs:msg/DepthActivated.idl
// generated code does not contain a copyright notice

#ifndef MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__BUILDER_HPP_
#define MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__BUILDER_HPP_

#include "my_msgs/msg/detail/depth_activated__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace my_msgs
{

namespace msg
{

namespace builder
{

class Init_DepthActivated_depth_activated
{
public:
  Init_DepthActivated_depth_activated()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_msgs::msg::DepthActivated depth_activated(::my_msgs::msg::DepthActivated::_depth_activated_type arg)
  {
    msg_.depth_activated = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::msg::DepthActivated msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::msg::DepthActivated>()
{
  return my_msgs::msg::builder::Init_DepthActivated_depth_activated();
}

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__BUILDER_HPP_
