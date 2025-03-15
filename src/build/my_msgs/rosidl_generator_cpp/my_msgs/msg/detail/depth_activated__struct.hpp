// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_msgs:msg/DepthActivated.idl
// generated code does not contain a copyright notice

#ifndef MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__STRUCT_HPP_
#define MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__my_msgs__msg__DepthActivated __attribute__((deprecated))
#else
# define DEPRECATED__my_msgs__msg__DepthActivated __declspec(deprecated)
#endif

namespace my_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DepthActivated_
{
  using Type = DepthActivated_<ContainerAllocator>;

  explicit DepthActivated_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->depth_activated = false;
    }
  }

  explicit DepthActivated_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->depth_activated = false;
    }
  }

  // field types and members
  using _depth_activated_type =
    bool;
  _depth_activated_type depth_activated;

  // setters for named parameter idiom
  Type & set__depth_activated(
    const bool & _arg)
  {
    this->depth_activated = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_msgs::msg::DepthActivated_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_msgs::msg::DepthActivated_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_msgs::msg::DepthActivated_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_msgs::msg::DepthActivated_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::DepthActivated_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::DepthActivated_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::DepthActivated_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::DepthActivated_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_msgs::msg::DepthActivated_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_msgs::msg::DepthActivated_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_msgs__msg__DepthActivated
    std::shared_ptr<my_msgs::msg::DepthActivated_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_msgs__msg__DepthActivated
    std::shared_ptr<my_msgs::msg::DepthActivated_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DepthActivated_ & other) const
  {
    if (this->depth_activated != other.depth_activated) {
      return false;
    }
    return true;
  }
  bool operator!=(const DepthActivated_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DepthActivated_

// alias to use template instance with default allocator
using DepthActivated =
  my_msgs::msg::DepthActivated_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__DEPTH_ACTIVATED__STRUCT_HPP_
