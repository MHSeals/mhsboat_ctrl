// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from boat_interfaces:msg/AiOutput.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__BUILDER_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "boat_interfaces/msg/detail/ai_output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace boat_interfaces
{

namespace msg
{

namespace builder
{

class Init_AiOutput_heights
{
public:
  explicit Init_AiOutput_heights(::boat_interfaces::msg::AiOutput & msg)
  : msg_(msg)
  {}
  ::boat_interfaces::msg::AiOutput heights(::boat_interfaces::msg::AiOutput::_heights_type arg)
  {
    msg_.heights = std::move(arg);
    return std::move(msg_);
  }

private:
  ::boat_interfaces::msg::AiOutput msg_;
};

class Init_AiOutput_widths
{
public:
  explicit Init_AiOutput_widths(::boat_interfaces::msg::AiOutput & msg)
  : msg_(msg)
  {}
  Init_AiOutput_heights widths(::boat_interfaces::msg::AiOutput::_widths_type arg)
  {
    msg_.widths = std::move(arg);
    return Init_AiOutput_heights(msg_);
  }

private:
  ::boat_interfaces::msg::AiOutput msg_;
};

class Init_AiOutput_tops
{
public:
  explicit Init_AiOutput_tops(::boat_interfaces::msg::AiOutput & msg)
  : msg_(msg)
  {}
  Init_AiOutput_widths tops(::boat_interfaces::msg::AiOutput::_tops_type arg)
  {
    msg_.tops = std::move(arg);
    return Init_AiOutput_widths(msg_);
  }

private:
  ::boat_interfaces::msg::AiOutput msg_;
};

class Init_AiOutput_lefts
{
public:
  explicit Init_AiOutput_lefts(::boat_interfaces::msg::AiOutput & msg)
  : msg_(msg)
  {}
  Init_AiOutput_tops lefts(::boat_interfaces::msg::AiOutput::_lefts_type arg)
  {
    msg_.lefts = std::move(arg);
    return Init_AiOutput_tops(msg_);
  }

private:
  ::boat_interfaces::msg::AiOutput msg_;
};

class Init_AiOutput_confidences
{
public:
  explicit Init_AiOutput_confidences(::boat_interfaces::msg::AiOutput & msg)
  : msg_(msg)
  {}
  Init_AiOutput_lefts confidences(::boat_interfaces::msg::AiOutput::_confidences_type arg)
  {
    msg_.confidences = std::move(arg);
    return Init_AiOutput_lefts(msg_);
  }

private:
  ::boat_interfaces::msg::AiOutput msg_;
};

class Init_AiOutput_types
{
public:
  explicit Init_AiOutput_types(::boat_interfaces::msg::AiOutput & msg)
  : msg_(msg)
  {}
  Init_AiOutput_confidences types(::boat_interfaces::msg::AiOutput::_types_type arg)
  {
    msg_.types = std::move(arg);
    return Init_AiOutput_confidences(msg_);
  }

private:
  ::boat_interfaces::msg::AiOutput msg_;
};

class Init_AiOutput_img_height
{
public:
  explicit Init_AiOutput_img_height(::boat_interfaces::msg::AiOutput & msg)
  : msg_(msg)
  {}
  Init_AiOutput_types img_height(::boat_interfaces::msg::AiOutput::_img_height_type arg)
  {
    msg_.img_height = std::move(arg);
    return Init_AiOutput_types(msg_);
  }

private:
  ::boat_interfaces::msg::AiOutput msg_;
};

class Init_AiOutput_img_width
{
public:
  explicit Init_AiOutput_img_width(::boat_interfaces::msg::AiOutput & msg)
  : msg_(msg)
  {}
  Init_AiOutput_img_height img_width(::boat_interfaces::msg::AiOutput::_img_width_type arg)
  {
    msg_.img_width = std::move(arg);
    return Init_AiOutput_img_height(msg_);
  }

private:
  ::boat_interfaces::msg::AiOutput msg_;
};

class Init_AiOutput_num
{
public:
  Init_AiOutput_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AiOutput_img_width num(::boat_interfaces::msg::AiOutput::_num_type arg)
  {
    msg_.num = std::move(arg);
    return Init_AiOutput_img_width(msg_);
  }

private:
  ::boat_interfaces::msg::AiOutput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::boat_interfaces::msg::AiOutput>()
{
  return boat_interfaces::msg::builder::Init_AiOutput_num();
}

}  // namespace boat_interfaces

#endif  // BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__BUILDER_HPP_
