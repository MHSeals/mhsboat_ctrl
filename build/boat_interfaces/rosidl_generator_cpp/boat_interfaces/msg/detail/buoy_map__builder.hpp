// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from boat_interfaces:msg/BuoyMap.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BUOY_MAP__BUILDER_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__BUOY_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "boat_interfaces/msg/detail/buoy_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace boat_interfaces
{

namespace msg
{

namespace builder
{

class Init_BuoyMap_colors
{
public:
  explicit Init_BuoyMap_colors(::boat_interfaces::msg::BuoyMap & msg)
  : msg_(msg)
  {}
  ::boat_interfaces::msg::BuoyMap colors(::boat_interfaces::msg::BuoyMap::_colors_type arg)
  {
    msg_.colors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::boat_interfaces::msg::BuoyMap msg_;
};

class Init_BuoyMap_types
{
public:
  explicit Init_BuoyMap_types(::boat_interfaces::msg::BuoyMap & msg)
  : msg_(msg)
  {}
  Init_BuoyMap_colors types(::boat_interfaces::msg::BuoyMap::_types_type arg)
  {
    msg_.types = std::move(arg);
    return Init_BuoyMap_colors(msg_);
  }

private:
  ::boat_interfaces::msg::BuoyMap msg_;
};

class Init_BuoyMap_z
{
public:
  explicit Init_BuoyMap_z(::boat_interfaces::msg::BuoyMap & msg)
  : msg_(msg)
  {}
  Init_BuoyMap_types z(::boat_interfaces::msg::BuoyMap::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_BuoyMap_types(msg_);
  }

private:
  ::boat_interfaces::msg::BuoyMap msg_;
};

class Init_BuoyMap_y
{
public:
  explicit Init_BuoyMap_y(::boat_interfaces::msg::BuoyMap & msg)
  : msg_(msg)
  {}
  Init_BuoyMap_z y(::boat_interfaces::msg::BuoyMap::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_BuoyMap_z(msg_);
  }

private:
  ::boat_interfaces::msg::BuoyMap msg_;
};

class Init_BuoyMap_x
{
public:
  Init_BuoyMap_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BuoyMap_y x(::boat_interfaces::msg::BuoyMap::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_BuoyMap_y(msg_);
  }

private:
  ::boat_interfaces::msg::BuoyMap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::boat_interfaces::msg::BuoyMap>()
{
  return boat_interfaces::msg::builder::Init_BuoyMap_x();
}

}  // namespace boat_interfaces

#endif  // BOAT_INTERFACES__MSG__DETAIL__BUOY_MAP__BUILDER_HPP_
