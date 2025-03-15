// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_msgs:msg/YoloDetection.idl
// generated code does not contain a copyright notice

#ifndef MY_MSGS__MSG__DETAIL__YOLO_DETECTION__BUILDER_HPP_
#define MY_MSGS__MSG__DETAIL__YOLO_DETECTION__BUILDER_HPP_

#include "my_msgs/msg/detail/yolo_detection__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace my_msgs
{

namespace msg
{

namespace builder
{

class Init_YoloDetection_ymin
{
public:
  explicit Init_YoloDetection_ymin(::my_msgs::msg::YoloDetection & msg)
  : msg_(msg)
  {}
  ::my_msgs::msg::YoloDetection ymin(::my_msgs::msg::YoloDetection::_ymin_type arg)
  {
    msg_.ymin = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::msg::YoloDetection msg_;
};

class Init_YoloDetection_xmin
{
public:
  explicit Init_YoloDetection_xmin(::my_msgs::msg::YoloDetection & msg)
  : msg_(msg)
  {}
  Init_YoloDetection_ymin xmin(::my_msgs::msg::YoloDetection::_xmin_type arg)
  {
    msg_.xmin = std::move(arg);
    return Init_YoloDetection_ymin(msg_);
  }

private:
  ::my_msgs::msg::YoloDetection msg_;
};

class Init_YoloDetection_ymax
{
public:
  explicit Init_YoloDetection_ymax(::my_msgs::msg::YoloDetection & msg)
  : msg_(msg)
  {}
  Init_YoloDetection_xmin ymax(::my_msgs::msg::YoloDetection::_ymax_type arg)
  {
    msg_.ymax = std::move(arg);
    return Init_YoloDetection_xmin(msg_);
  }

private:
  ::my_msgs::msg::YoloDetection msg_;
};

class Init_YoloDetection_xmax
{
public:
  explicit Init_YoloDetection_xmax(::my_msgs::msg::YoloDetection & msg)
  : msg_(msg)
  {}
  Init_YoloDetection_ymax xmax(::my_msgs::msg::YoloDetection::_xmax_type arg)
  {
    msg_.xmax = std::move(arg);
    return Init_YoloDetection_ymax(msg_);
  }

private:
  ::my_msgs::msg::YoloDetection msg_;
};

class Init_YoloDetection_screen_height
{
public:
  explicit Init_YoloDetection_screen_height(::my_msgs::msg::YoloDetection & msg)
  : msg_(msg)
  {}
  Init_YoloDetection_xmax screen_height(::my_msgs::msg::YoloDetection::_screen_height_type arg)
  {
    msg_.screen_height = std::move(arg);
    return Init_YoloDetection_xmax(msg_);
  }

private:
  ::my_msgs::msg::YoloDetection msg_;
};

class Init_YoloDetection_screen_width
{
public:
  explicit Init_YoloDetection_screen_width(::my_msgs::msg::YoloDetection & msg)
  : msg_(msg)
  {}
  Init_YoloDetection_screen_height screen_width(::my_msgs::msg::YoloDetection::_screen_width_type arg)
  {
    msg_.screen_width = std::move(arg);
    return Init_YoloDetection_screen_height(msg_);
  }

private:
  ::my_msgs::msg::YoloDetection msg_;
};

class Init_YoloDetection_label
{
public:
  Init_YoloDetection_label()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YoloDetection_screen_width label(::my_msgs::msg::YoloDetection::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_YoloDetection_screen_width(msg_);
  }

private:
  ::my_msgs::msg::YoloDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::msg::YoloDetection>()
{
  return my_msgs::msg::builder::Init_YoloDetection_label();
}

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__YOLO_DETECTION__BUILDER_HPP_
