// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#ifndef MY_MSGS__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_
#define MY_MSGS__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_

#include "my_msgs/msg/detail/vehicle_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace my_msgs
{

namespace msg
{

namespace builder
{

class Init_VehicleState_substate
{
public:
  explicit Init_VehicleState_substate(::my_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  ::my_msgs::msg::VehicleState substate(::my_msgs::msg::VehicleState::_substate_type arg)
  {
    msg_.substate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_state
{
public:
  Init_VehicleState_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleState_substate state(::my_msgs::msg::VehicleState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_VehicleState_substate(msg_);
  }

private:
  ::my_msgs::msg::VehicleState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::msg::VehicleState>()
{
  return my_msgs::msg::builder::Init_VehicleState_state();
}

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_
