// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from boat_interfaces:msg/BuoyMap.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BUOY_MAP__STRUCT_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__BUOY_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__boat_interfaces__msg__BuoyMap __attribute__((deprecated))
#else
# define DEPRECATED__boat_interfaces__msg__BuoyMap __declspec(deprecated)
#endif

namespace boat_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BuoyMap_
{
  using Type = BuoyMap_<ContainerAllocator>;

  explicit BuoyMap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit BuoyMap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x_type x;
  using _y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_type y;
  using _z_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _z_type z;
  using _types_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _types_type types;
  using _colors_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _colors_type colors;

  // setters for named parameter idiom
  Type & set__x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__types(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->types = _arg;
    return *this;
  }
  Type & set__colors(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->colors = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    boat_interfaces::msg::BuoyMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const boat_interfaces::msg::BuoyMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<boat_interfaces::msg::BuoyMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<boat_interfaces::msg::BuoyMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      boat_interfaces::msg::BuoyMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<boat_interfaces::msg::BuoyMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      boat_interfaces::msg::BuoyMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<boat_interfaces::msg::BuoyMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<boat_interfaces::msg::BuoyMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<boat_interfaces::msg::BuoyMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__boat_interfaces__msg__BuoyMap
    std::shared_ptr<boat_interfaces::msg::BuoyMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__boat_interfaces__msg__BuoyMap
    std::shared_ptr<boat_interfaces::msg::BuoyMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BuoyMap_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->types != other.types) {
      return false;
    }
    if (this->colors != other.colors) {
      return false;
    }
    return true;
  }
  bool operator!=(const BuoyMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BuoyMap_

// alias to use template instance with default allocator
using BuoyMap =
  boat_interfaces::msg::BuoyMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace boat_interfaces

#endif  // BOAT_INTERFACES__MSG__DETAIL__BUOY_MAP__STRUCT_HPP_
