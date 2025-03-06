// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from boat_interfaces:msg/BuoyCoordinates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "boat_interfaces/msg/detail/buoy_coordinates__rosidl_typesupport_introspection_c.h"
#include "boat_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "boat_interfaces/msg/detail/buoy_coordinates__functions.h"
#include "boat_interfaces/msg/detail/buoy_coordinates__struct.h"


// Include directives for member types
// Member `latitudes`
// Member `longitudes`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `types`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  boat_interfaces__msg__BuoyCoordinates__init(message_memory);
}

void boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_fini_function(void * message_memory)
{
  boat_interfaces__msg__BuoyCoordinates__fini(message_memory);
}

size_t boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__size_function__BuoyCoordinates__latitudes(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_const_function__BuoyCoordinates__latitudes(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_function__BuoyCoordinates__latitudes(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__fetch_function__BuoyCoordinates__latitudes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_const_function__BuoyCoordinates__latitudes(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__assign_function__BuoyCoordinates__latitudes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_function__BuoyCoordinates__latitudes(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__resize_function__BuoyCoordinates__latitudes(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__size_function__BuoyCoordinates__longitudes(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_const_function__BuoyCoordinates__longitudes(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_function__BuoyCoordinates__longitudes(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__fetch_function__BuoyCoordinates__longitudes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_const_function__BuoyCoordinates__longitudes(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__assign_function__BuoyCoordinates__longitudes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_function__BuoyCoordinates__longitudes(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__resize_function__BuoyCoordinates__longitudes(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__size_function__BuoyCoordinates__types(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_const_function__BuoyCoordinates__types(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_function__BuoyCoordinates__types(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__fetch_function__BuoyCoordinates__types(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_const_function__BuoyCoordinates__types(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__assign_function__BuoyCoordinates__types(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_function__BuoyCoordinates__types(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__resize_function__BuoyCoordinates__types(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_message_member_array[3] = {
  {
    "latitudes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__BuoyCoordinates, latitudes),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__size_function__BuoyCoordinates__latitudes,  // size() function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_const_function__BuoyCoordinates__latitudes,  // get_const(index) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_function__BuoyCoordinates__latitudes,  // get(index) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__fetch_function__BuoyCoordinates__latitudes,  // fetch(index, &value) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__assign_function__BuoyCoordinates__latitudes,  // assign(index, value) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__resize_function__BuoyCoordinates__latitudes  // resize(index) function pointer
  },
  {
    "longitudes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__BuoyCoordinates, longitudes),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__size_function__BuoyCoordinates__longitudes,  // size() function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_const_function__BuoyCoordinates__longitudes,  // get_const(index) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_function__BuoyCoordinates__longitudes,  // get(index) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__fetch_function__BuoyCoordinates__longitudes,  // fetch(index, &value) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__assign_function__BuoyCoordinates__longitudes,  // assign(index, value) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__resize_function__BuoyCoordinates__longitudes  // resize(index) function pointer
  },
  {
    "types",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__BuoyCoordinates, types),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__size_function__BuoyCoordinates__types,  // size() function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_const_function__BuoyCoordinates__types,  // get_const(index) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__get_function__BuoyCoordinates__types,  // get(index) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__fetch_function__BuoyCoordinates__types,  // fetch(index, &value) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__assign_function__BuoyCoordinates__types,  // assign(index, value) function pointer
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__resize_function__BuoyCoordinates__types  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_message_members = {
  "boat_interfaces__msg",  // message namespace
  "BuoyCoordinates",  // message name
  3,  // number of fields
  sizeof(boat_interfaces__msg__BuoyCoordinates),
  boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_message_member_array,  // message members
  boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_init_function,  // function to initialize message memory (memory has to be allocated)
  boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_message_type_support_handle = {
  0,
  &boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_boat_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, boat_interfaces, msg, BuoyCoordinates)() {
  if (!boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_message_type_support_handle.typesupport_identifier) {
    boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &boat_interfaces__msg__BuoyCoordinates__rosidl_typesupport_introspection_c__BuoyCoordinates_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
