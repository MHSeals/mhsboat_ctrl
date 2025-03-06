// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from boat_interfaces:msg/BuoyCoordinates.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__TRAITS_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "boat_interfaces/msg/detail/buoy_coordinates__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace boat_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BuoyCoordinates & msg,
  std::ostream & out)
{
  out << "{";
  // member: latitudes
  {
    if (msg.latitudes.size() == 0) {
      out << "latitudes: []";
    } else {
      out << "latitudes: [";
      size_t pending_items = msg.latitudes.size();
      for (auto item : msg.latitudes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: longitudes
  {
    if (msg.longitudes.size() == 0) {
      out << "longitudes: []";
    } else {
      out << "longitudes: [";
      size_t pending_items = msg.longitudes.size();
      for (auto item : msg.longitudes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: types
  {
    if (msg.types.size() == 0) {
      out << "types: []";
    } else {
      out << "types: [";
      size_t pending_items = msg.types.size();
      for (auto item : msg.types) {
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
  const BuoyCoordinates & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: latitudes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.latitudes.size() == 0) {
      out << "latitudes: []\n";
    } else {
      out << "latitudes:\n";
      for (auto item : msg.latitudes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: longitudes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.longitudes.size() == 0) {
      out << "longitudes: []\n";
    } else {
      out << "longitudes:\n";
      for (auto item : msg.longitudes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: types
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.types.size() == 0) {
      out << "types: []\n";
    } else {
      out << "types:\n";
      for (auto item : msg.types) {
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

inline std::string to_yaml(const BuoyCoordinates & msg, bool use_flow_style = false)
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
  const boat_interfaces::msg::BuoyCoordinates & msg,
  std::ostream & out, size_t indentation = 0)
{
  boat_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use boat_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const boat_interfaces::msg::BuoyCoordinates & msg)
{
  return boat_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<boat_interfaces::msg::BuoyCoordinates>()
{
  return "boat_interfaces::msg::BuoyCoordinates";
}

template<>
inline const char * name<boat_interfaces::msg::BuoyCoordinates>()
{
  return "boat_interfaces/msg/BuoyCoordinates";
}

template<>
struct has_fixed_size<boat_interfaces::msg::BuoyCoordinates>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<boat_interfaces::msg::BuoyCoordinates>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<boat_interfaces::msg::BuoyCoordinates>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__TRAITS_HPP_
