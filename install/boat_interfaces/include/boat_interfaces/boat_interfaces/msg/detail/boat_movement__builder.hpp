// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from boat_interfaces:msg/BoatMovement.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__BUILDER_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "boat_interfaces/msg/detail/boat_movement__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace boat_interfaces
{

namespace msg
{

namespace builder
{

class Init_BoatMovement_dzr
{
public:
  explicit Init_BoatMovement_dzr(::boat_interfaces::msg::BoatMovement & msg)
  : msg_(msg)
  {}
  ::boat_interfaces::msg::BoatMovement dzr(::boat_interfaces::msg::BoatMovement::_dzr_type arg)
  {
    msg_.dzr = std::move(arg);
    return std::move(msg_);
  }

private:
  ::boat_interfaces::msg::BoatMovement msg_;
};

class Init_BoatMovement_dy
{
public:
  explicit Init_BoatMovement_dy(::boat_interfaces::msg::BoatMovement & msg)
  : msg_(msg)
  {}
  Init_BoatMovement_dzr dy(::boat_interfaces::msg::BoatMovement::_dy_type arg)
  {
    msg_.dy = std::move(arg);
    return Init_BoatMovement_dzr(msg_);
  }

private:
  ::boat_interfaces::msg::BoatMovement msg_;
};

class Init_BoatMovement_dx
{
public:
  Init_BoatMovement_dx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoatMovement_dy dx(::boat_interfaces::msg::BoatMovement::_dx_type arg)
  {
    msg_.dx = std::move(arg);
    return Init_BoatMovement_dy(msg_);
  }

private:
  ::boat_interfaces::msg::BoatMovement msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::boat_interfaces::msg::BoatMovement>()
{
  return boat_interfaces::msg::builder::Init_BoatMovement_dx();
}

}  // namespace boat_interfaces

#endif  // BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__BUILDER_HPP_
