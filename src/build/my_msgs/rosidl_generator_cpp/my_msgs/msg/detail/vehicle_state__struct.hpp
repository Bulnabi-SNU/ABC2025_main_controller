// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#ifndef MY_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_HPP_
#define MY_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__my_msgs__msg__VehicleState __attribute__((deprecated))
#else
# define DEPRECATED__my_msgs__msg__VehicleState __declspec(deprecated)
#endif

namespace my_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehicleState_
{
  using Type = VehicleState_<ContainerAllocator>;

  explicit VehicleState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->substate = "";
    }
  }

  explicit VehicleState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_alloc),
    substate(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->substate = "";
    }
  }

  // field types and members
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _state_type state;
  using _substate_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _substate_type substate;

  // setters for named parameter idiom
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__substate(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->substate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_msgs::msg::VehicleState_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_msgs::msg::VehicleState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_msgs::msg::VehicleState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_msgs::msg::VehicleState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::VehicleState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::VehicleState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::VehicleState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::VehicleState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_msgs::msg::VehicleState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_msgs::msg::VehicleState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_msgs__msg__VehicleState
    std::shared_ptr<my_msgs::msg::VehicleState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_msgs__msg__VehicleState
    std::shared_ptr<my_msgs::msg::VehicleState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleState_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    if (this->substate != other.substate) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleState_

// alias to use template instance with default allocator
using VehicleState =
  my_msgs::msg::VehicleState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_HPP_
