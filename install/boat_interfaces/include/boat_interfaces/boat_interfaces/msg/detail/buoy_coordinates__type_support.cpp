// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from boat_interfaces:msg/BuoyCoordinates.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "boat_interfaces/msg/detail/buoy_coordinates__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace boat_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BuoyCoordinates_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) boat_interfaces::msg::BuoyCoordinates(_init);
}

void BuoyCoordinates_fini_function(void * message_memory)
{
  auto typed_message = static_cast<boat_interfaces::msg::BuoyCoordinates *>(message_memory);
  typed_message->~BuoyCoordinates();
}

size_t size_function__BuoyCoordinates__latitudes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BuoyCoordinates__latitudes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__BuoyCoordinates__latitudes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__BuoyCoordinates__latitudes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BuoyCoordinates__latitudes(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BuoyCoordinates__latitudes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BuoyCoordinates__latitudes(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__BuoyCoordinates__latitudes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BuoyCoordinates__longitudes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BuoyCoordinates__longitudes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__BuoyCoordinates__longitudes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__BuoyCoordinates__longitudes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BuoyCoordinates__longitudes(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BuoyCoordinates__longitudes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BuoyCoordinates__longitudes(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__BuoyCoordinates__longitudes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BuoyCoordinates__types(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BuoyCoordinates__types(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__BuoyCoordinates__types(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__BuoyCoordinates__types(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__BuoyCoordinates__types(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__BuoyCoordinates__types(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__BuoyCoordinates__types(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__BuoyCoordinates__types(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BuoyCoordinates_message_member_array[3] = {
  {
    "latitudes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces::msg::BuoyCoordinates, latitudes),  // bytes offset in struct
    nullptr,  // default value
    size_function__BuoyCoordinates__latitudes,  // size() function pointer
    get_const_function__BuoyCoordinates__latitudes,  // get_const(index) function pointer
    get_function__BuoyCoordinates__latitudes,  // get(index) function pointer
    fetch_function__BuoyCoordinates__latitudes,  // fetch(index, &value) function pointer
    assign_function__BuoyCoordinates__latitudes,  // assign(index, value) function pointer
    resize_function__BuoyCoordinates__latitudes  // resize(index) function pointer
  },
  {
    "longitudes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces::msg::BuoyCoordinates, longitudes),  // bytes offset in struct
    nullptr,  // default value
    size_function__BuoyCoordinates__longitudes,  // size() function pointer
    get_const_function__BuoyCoordinates__longitudes,  // get_const(index) function pointer
    get_function__BuoyCoordinates__longitudes,  // get(index) function pointer
    fetch_function__BuoyCoordinates__longitudes,  // fetch(index, &value) function pointer
    assign_function__BuoyCoordinates__longitudes,  // assign(index, value) function pointer
    resize_function__BuoyCoordinates__longitudes  // resize(index) function pointer
  },
  {
    "types",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces::msg::BuoyCoordinates, types),  // bytes offset in struct
    nullptr,  // default value
    size_function__BuoyCoordinates__types,  // size() function pointer
    get_const_function__BuoyCoordinates__types,  // get_const(index) function pointer
    get_function__BuoyCoordinates__types,  // get(index) function pointer
    fetch_function__BuoyCoordinates__types,  // fetch(index, &value) function pointer
    assign_function__BuoyCoordinates__types,  // assign(index, value) function pointer
    resize_function__BuoyCoordinates__types  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BuoyCoordinates_message_members = {
  "boat_interfaces::msg",  // message namespace
  "BuoyCoordinates",  // message name
  3,  // number of fields
  sizeof(boat_interfaces::msg::BuoyCoordinates),
  BuoyCoordinates_message_member_array,  // message members
  BuoyCoordinates_init_function,  // function to initialize message memory (memory has to be allocated)
  BuoyCoordinates_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BuoyCoordinates_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BuoyCoordinates_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace boat_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<boat_interfaces::msg::BuoyCoordinates>()
{
  return &::boat_interfaces::msg::rosidl_typesupport_introspection_cpp::BuoyCoordinates_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, boat_interfaces, msg, BuoyCoordinates)() {
  return &::boat_interfaces::msg::rosidl_typesupport_introspection_cpp::BuoyCoordinates_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
