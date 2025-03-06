// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from boat_interfaces:msg/AiOutput.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__STRUCT_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__boat_interfaces__msg__AiOutput __attribute__((deprecated))
#else
# define DEPRECATED__boat_interfaces__msg__AiOutput __declspec(deprecated)
#endif

namespace boat_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AiOutput_
{
  using Type = AiOutput_<ContainerAllocator>;

  explicit AiOutput_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0l;
      this->img_width = 0l;
      this->img_height = 0l;
    }
  }

  explicit AiOutput_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0l;
      this->img_width = 0l;
      this->img_height = 0l;
    }
  }

  // field types and members
  using _num_type =
    int32_t;
  _num_type num;
  using _img_width_type =
    int32_t;
  _img_width_type img_width;
  using _img_height_type =
    int32_t;
  _img_height_type img_height;
  using _types_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _types_type types;
  using _confidences_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _confidences_type confidences;
  using _lefts_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _lefts_type lefts;
  using _tops_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _tops_type tops;
  using _widths_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _widths_type widths;
  using _heights_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _heights_type heights;

  // setters for named parameter idiom
  Type & set__num(
    const int32_t & _arg)
  {
    this->num = _arg;
    return *this;
  }
  Type & set__img_width(
    const int32_t & _arg)
  {
    this->img_width = _arg;
    return *this;
  }
  Type & set__img_height(
    const int32_t & _arg)
  {
    this->img_height = _arg;
    return *this;
  }
  Type & set__types(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->types = _arg;
    return *this;
  }
  Type & set__confidences(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->confidences = _arg;
    return *this;
  }
  Type & set__lefts(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->lefts = _arg;
    return *this;
  }
  Type & set__tops(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->tops = _arg;
    return *this;
  }
  Type & set__widths(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->widths = _arg;
    return *this;
  }
  Type & set__heights(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->heights = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    boat_interfaces::msg::AiOutput_<ContainerAllocator> *;
  using ConstRawPtr =
    const boat_interfaces::msg::AiOutput_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<boat_interfaces::msg::AiOutput_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<boat_interfaces::msg::AiOutput_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      boat_interfaces::msg::AiOutput_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<boat_interfaces::msg::AiOutput_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      boat_interfaces::msg::AiOutput_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<boat_interfaces::msg::AiOutput_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<boat_interfaces::msg::AiOutput_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<boat_interfaces::msg::AiOutput_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__boat_interfaces__msg__AiOutput
    std::shared_ptr<boat_interfaces::msg::AiOutput_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__boat_interfaces__msg__AiOutput
    std::shared_ptr<boat_interfaces::msg::AiOutput_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AiOutput_ & other) const
  {
    if (this->num != other.num) {
      return false;
    }
    if (this->img_width != other.img_width) {
      return false;
    }
    if (this->img_height != other.img_height) {
      return false;
    }
    if (this->types != other.types) {
      return false;
    }
    if (this->confidences != other.confidences) {
      return false;
    }
    if (this->lefts != other.lefts) {
      return false;
    }
    if (this->tops != other.tops) {
      return false;
    }
    if (this->widths != other.widths) {
      return false;
    }
    if (this->heights != other.heights) {
      return false;
    }
    return true;
  }
  bool operator!=(const AiOutput_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AiOutput_

// alias to use template instance with default allocator
using AiOutput =
  boat_interfaces::msg::AiOutput_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace boat_interfaces

#endif  // BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__STRUCT_HPP_
