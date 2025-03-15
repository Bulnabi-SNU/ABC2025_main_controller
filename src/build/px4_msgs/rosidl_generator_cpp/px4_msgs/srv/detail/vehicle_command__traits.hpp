// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:srv/VehicleCommand.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__TRAITS_HPP_
#define PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__TRAITS_HPP_

#include "px4_msgs/srv/detail/vehicle_command__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'request'
#include "px4_msgs/msg/detail/vehicle_command__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<px4_msgs::srv::VehicleCommand_Request>()
{
  return "px4_msgs::srv::VehicleCommand_Request";
}

template<>
inline const char * name<px4_msgs::srv::VehicleCommand_Request>()
{
  return "px4_msgs/srv/VehicleCommand_Request";
}

template<>
struct has_fixed_size<px4_msgs::srv::VehicleCommand_Request>
  : std::integral_constant<bool, has_fixed_size<px4_msgs::msg::VehicleCommand>::value> {};

template<>
struct has_bounded_size<px4_msgs::srv::VehicleCommand_Request>
  : std::integral_constant<bool, has_bounded_size<px4_msgs::msg::VehicleCommand>::value> {};

template<>
struct is_message<px4_msgs::srv::VehicleCommand_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'reply'
#include "px4_msgs/msg/detail/vehicle_command_ack__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<px4_msgs::srv::VehicleCommand_Response>()
{
  return "px4_msgs::srv::VehicleCommand_Response";
}

template<>
inline const char * name<px4_msgs::srv::VehicleCommand_Response>()
{
  return "px4_msgs/srv/VehicleCommand_Response";
}

template<>
struct has_fixed_size<px4_msgs::srv::VehicleCommand_Response>
  : std::integral_constant<bool, has_fixed_size<px4_msgs::msg::VehicleCommandAck>::value> {};

template<>
struct has_bounded_size<px4_msgs::srv::VehicleCommand_Response>
  : std::integral_constant<bool, has_bounded_size<px4_msgs::msg::VehicleCommandAck>::value> {};

template<>
struct is_message<px4_msgs::srv::VehicleCommand_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<px4_msgs::srv::VehicleCommand>()
{
  return "px4_msgs::srv::VehicleCommand";
}

template<>
inline const char * name<px4_msgs::srv::VehicleCommand>()
{
  return "px4_msgs/srv/VehicleCommand";
}

template<>
struct has_fixed_size<px4_msgs::srv::VehicleCommand>
  : std::integral_constant<
    bool,
    has_fixed_size<px4_msgs::srv::VehicleCommand_Request>::value &&
    has_fixed_size<px4_msgs::srv::VehicleCommand_Response>::value
  >
{
};

template<>
struct has_bounded_size<px4_msgs::srv::VehicleCommand>
  : std::integral_constant<
    bool,
    has_bounded_size<px4_msgs::srv::VehicleCommand_Request>::value &&
    has_bounded_size<px4_msgs::srv::VehicleCommand_Response>::value
  >
{
};

template<>
struct is_service<px4_msgs::srv::VehicleCommand>
  : std::true_type
{
};

template<>
struct is_service_request<px4_msgs::srv::VehicleCommand_Request>
  : std::true_type
{
};

template<>
struct is_service_response<px4_msgs::srv::VehicleCommand_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__TRAITS_HPP_
