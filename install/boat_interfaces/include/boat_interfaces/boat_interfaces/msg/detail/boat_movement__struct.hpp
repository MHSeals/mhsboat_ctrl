// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from boat_interfaces:msg/BoatMovement.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__STRUCT_HPP_
#define BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__boat_interfaces__msg__BoatMovement __attribute__((deprecated))
#else
# define DEPRECATED__boat_interfaces__msg__BoatMovement __declspec(deprecated)
#endif

namespace boat_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoatMovement_
{
  using Type = BoatMovement_<ContainerAllocator>;

  explicit BoatMovement_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit BoatMovement_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _dx_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _dx_type dx;
  using _dy_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _dy_type dy;
  using _dzr_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _dzr_type dzr;

  // setters for named parameter idiom
  Type & set__dx(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->dx = _arg;
    return *this;
  }
  Type & set__dy(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->dy = _arg;
    return *this;
  }
  Type & set__dzr(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->dzr = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    boat_interfaces::msg::BoatMovement_<ContainerAllocator> *;
  using ConstRawPtr =
    const boat_interfaces::msg::BoatMovement_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<boat_interfaces::msg::BoatMovement_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<boat_interfaces::msg::BoatMovement_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      boat_interfaces::msg::BoatMovement_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<boat_interfaces::msg::BoatMovement_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      boat_interfaces::msg::BoatMovement_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<boat_interfaces::msg::BoatMovement_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<boat_interfaces::msg::BoatMovement_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<boat_interfaces::msg::BoatMovement_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__boat_interfaces__msg__BoatMovement
    std::shared_ptr<boat_interfaces::msg::BoatMovement_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__boat_interfaces__msg__BoatMovement
    std::shared_ptr<boat_interfaces::msg::BoatMovement_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoatMovement_ & other) const
  {
    if (this->dx != other.dx) {
      return false;
    }
    if (this->dy != other.dy) {
      return false;
    }
    if (this->dzr != other.dzr) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoatMovement_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoatMovement_

// alias to use template instance with default allocator
using BoatMovement =
  boat_interfaces::msg::BoatMovement_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace boat_interfaces

#endif  // BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__STRUCT_HPP_
