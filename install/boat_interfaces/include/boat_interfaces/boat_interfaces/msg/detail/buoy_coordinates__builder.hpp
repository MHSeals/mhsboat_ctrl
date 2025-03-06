// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from boat_interfaces:msg/BuoyCoordinates.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__BUILDER_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "boat_interfaces/msg/detail/buoy_coordinates__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace boat_interfaces
{

namespace msg
{

namespace builder
{

class Init_BuoyCoordinates_types
{
public:
  explicit Init_BuoyCoordinates_types(::boat_interfaces::msg::BuoyCoordinates & msg)
  : msg_(msg)
  {}
  ::boat_interfaces::msg::BuoyCoordinates types(::boat_interfaces::msg::BuoyCoordinates::_types_type arg)
  {
    msg_.types = std::move(arg);
    return std::move(msg_);
  }

private:
  ::boat_interfaces::msg::BuoyCoordinates msg_;
};

class Init_BuoyCoordinates_longitudes
{
public:
  explicit Init_BuoyCoordinates_longitudes(::boat_interfaces::msg::BuoyCoordinates & msg)
  : msg_(msg)
  {}
  Init_BuoyCoordinates_types longitudes(::boat_interfaces::msg::BuoyCoordinates::_longitudes_type arg)
  {
    msg_.longitudes = std::move(arg);
    return Init_BuoyCoordinates_types(msg_);
  }

private:
  ::boat_interfaces::msg::BuoyCoordinates msg_;
};

class Init_BuoyCoordinates_latitudes
{
public:
  Init_BuoyCoordinates_latitudes()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BuoyCoordinates_longitudes latitudes(::boat_interfaces::msg::BuoyCoordinates::_latitudes_type arg)
  {
    msg_.latitudes = std::move(arg);
    return Init_BuoyCoordinates_longitudes(msg_);
  }

private:
  ::boat_interfaces::msg::BuoyCoordinates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::boat_interfaces::msg::BuoyCoordinates>()
{
  return boat_interfaces::msg::builder::Init_BuoyCoordinates_latitudes();
}

}  // namespace boat_interfaces

#endif  // BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__BUILDER_HPP_
