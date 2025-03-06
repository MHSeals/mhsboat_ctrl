// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from boat_interfaces:msg/BuoyCoordinates.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__STRUCT_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__boat_interfaces__msg__BuoyCoordinates __attribute__((deprecated))
#else
# define DEPRECATED__boat_interfaces__msg__BuoyCoordinates __declspec(deprecated)
#endif

namespace boat_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BuoyCoordinates_
{
  using Type = BuoyCoordinates_<ContainerAllocator>;

  explicit BuoyCoordinates_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit BuoyCoordinates_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _latitudes_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _latitudes_type latitudes;
  using _longitudes_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _longitudes_type longitudes;
  using _types_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _types_type types;

  // setters for named parameter idiom
  Type & set__latitudes(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->latitudes = _arg;
    return *this;
  }
  Type & set__longitudes(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->longitudes = _arg;
    return *this;
  }
  Type & set__types(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->types = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator> *;
  using ConstRawPtr =
    const boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__boat_interfaces__msg__BuoyCoordinates
    std::shared_ptr<boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__boat_interfaces__msg__BuoyCoordinates
    std::shared_ptr<boat_interfaces::msg::BuoyCoordinates_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BuoyCoordinates_ & other) const
  {
    if (this->latitudes != other.latitudes) {
      return false;
    }
    if (this->longitudes != other.longitudes) {
      return false;
    }
    if (this->types != other.types) {
      return false;
    }
    return true;
  }
  bool operator!=(const BuoyCoordinates_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BuoyCoordinates_

// alias to use template instance with default allocator
using BuoyCoordinates =
  boat_interfaces::msg::BuoyCoordinates_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace boat_interfaces

#endif  // BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__STRUCT_HPP_
