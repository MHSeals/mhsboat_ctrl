// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from boat_interfaces:msg/AiOutput.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__TRAITS_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "boat_interfaces/msg/detail/ai_output__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace boat_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const AiOutput & msg,
  std::ostream & out)
{
  out << "{";
  // member: num
  {
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
    out << ", ";
  }

  // member: img_width
  {
    out << "img_width: ";
    rosidl_generator_traits::value_to_yaml(msg.img_width, out);
    out << ", ";
  }

  // member: img_height
  {
    out << "img_height: ";
    rosidl_generator_traits::value_to_yaml(msg.img_height, out);
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
    out << ", ";
  }

  // member: confidences
  {
    if (msg.confidences.size() == 0) {
      out << "confidences: []";
    } else {
      out << "confidences: [";
      size_t pending_items = msg.confidences.size();
      for (auto item : msg.confidences) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: lefts
  {
    if (msg.lefts.size() == 0) {
      out << "lefts: []";
    } else {
      out << "lefts: [";
      size_t pending_items = msg.lefts.size();
      for (auto item : msg.lefts) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tops
  {
    if (msg.tops.size() == 0) {
      out << "tops: []";
    } else {
      out << "tops: [";
      size_t pending_items = msg.tops.size();
      for (auto item : msg.tops) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: widths
  {
    if (msg.widths.size() == 0) {
      out << "widths: []";
    } else {
      out << "widths: [";
      size_t pending_items = msg.widths.size();
      for (auto item : msg.widths) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: heights
  {
    if (msg.heights.size() == 0) {
      out << "heights: []";
    } else {
      out << "heights: [";
      size_t pending_items = msg.heights.size();
      for (auto item : msg.heights) {
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
  const AiOutput & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
    out << "\n";
  }

  // member: img_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "img_width: ";
    rosidl_generator_traits::value_to_yaml(msg.img_width, out);
    out << "\n";
  }

  // member: img_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "img_height: ";
    rosidl_generator_traits::value_to_yaml(msg.img_height, out);
    out << "\n";
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

  // member: confidences
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.confidences.size() == 0) {
      out << "confidences: []\n";
    } else {
      out << "confidences:\n";
      for (auto item : msg.confidences) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: lefts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.lefts.size() == 0) {
      out << "lefts: []\n";
    } else {
      out << "lefts:\n";
      for (auto item : msg.lefts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tops
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tops.size() == 0) {
      out << "tops: []\n";
    } else {
      out << "tops:\n";
      for (auto item : msg.tops) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: widths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.widths.size() == 0) {
      out << "widths: []\n";
    } else {
      out << "widths:\n";
      for (auto item : msg.widths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: heights
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.heights.size() == 0) {
      out << "heights: []\n";
    } else {
      out << "heights:\n";
      for (auto item : msg.heights) {
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

inline std::string to_yaml(const AiOutput & msg, bool use_flow_style = false)
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
  const boat_interfaces::msg::AiOutput & msg,
  std::ostream & out, size_t indentation = 0)
{
  boat_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use boat_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const boat_interfaces::msg::AiOutput & msg)
{
  return boat_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<boat_interfaces::msg::AiOutput>()
{
  return "boat_interfaces::msg::AiOutput";
}

template<>
inline const char * name<boat_interfaces::msg::AiOutput>()
{
  return "boat_interfaces/msg/AiOutput";
}

template<>
struct has_fixed_size<boat_interfaces::msg::AiOutput>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<boat_interfaces::msg::AiOutput>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<boat_interfaces::msg::AiOutput>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__TRAITS_HPP_
