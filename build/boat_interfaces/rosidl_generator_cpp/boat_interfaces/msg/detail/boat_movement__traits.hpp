// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from boat_interfaces:msg/BoatMovement.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__TRAITS_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "boat_interfaces/msg/detail/boat_movement__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace boat_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BoatMovement & msg,
  std::ostream & out)
{
  out << "{";
  // member: dx
  {
    if (msg.dx.size() == 0) {
      out << "dx: []";
    } else {
      out << "dx: [";
      size_t pending_items = msg.dx.size();
      for (auto item : msg.dx) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: dy
  {
    if (msg.dy.size() == 0) {
      out << "dy: []";
    } else {
      out << "dy: [";
      size_t pending_items = msg.dy.size();
      for (auto item : msg.dy) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: dzr
  {
    if (msg.dzr.size() == 0) {
      out << "dzr: []";
    } else {
      out << "dzr: [";
      size_t pending_items = msg.dzr.size();
      for (auto item : msg.dzr) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BoatMovement & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.dx.size() == 0) {
      out << "dx: []\n";
    } else {
      out << "dx:\n";
      for (auto item : msg.dx) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: dy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.dy.size() == 0) {
      out << "dy: []\n";
    } else {
      out << "dy:\n";
      for (auto item : msg.dy) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: dzr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.dzr.size() == 0) {
      out << "dzr: []\n";
    } else {
      out << "dzr:\n";
      for (auto item : msg.dzr) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BoatMovement & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace boat_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use boat_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const boat_interfaces::msg::BoatMovement & msg,
  std::ostream & out, size_t indentation = 0)
{
  boat_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use boat_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const boat_interfaces::msg::BoatMovement & msg)
{
  return boat_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<boat_interfaces::msg::BoatMovement>()
{
  return "boat_interfaces::msg::BoatMovement";
}

template<>
inline const char * name<boat_interfaces::msg::BoatMovement>()
{
  return "boat_interfaces/msg/BoatMovement";
}

template<>
struct has_fixed_size<boat_interfaces::msg::BoatMovement>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<boat_interfaces::msg::BoatMovement>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<boat_interfaces::msg::BoatMovement>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__TRAITS_HPP_
