// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from boat_interfaces:msg/BoatMovement.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "boat_interfaces/msg/detail/boat_movement__rosidl_typesupport_introspection_c.h"
#include "boat_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "boat_interfaces/msg/detail/boat_movement__functions.h"
#include "boat_interfaces/msg/detail/boat_movement__struct.h"


// Include directives for member types
// Member `dx`
// Member `dy`
// Member `dzr`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  boat_interfaces__msg__BoatMovement__init(message_memory);
}

void boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_fini_function(void * message_memory)
{
  boat_interfaces__msg__BoatMovement__fini(message_memory);
}

size_t boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__size_function__BoatMovement__dx(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_const_function__BoatMovement__dx(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_function__BoatMovement__dx(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__fetch_function__BoatMovement__dx(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_const_function__BoatMovement__dx(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__assign_function__BoatMovement__dx(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_function__BoatMovement__dx(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__resize_function__BoatMovement__dx(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__size_function__BoatMovement__dy(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_const_function__BoatMovement__dy(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_function__BoatMovement__dy(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__fetch_function__BoatMovement__dy(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_const_function__BoatMovement__dy(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__assign_function__BoatMovement__dy(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_function__BoatMovement__dy(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__resize_function__BoatMovement__dy(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__size_function__BoatMovement__dzr(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_const_function__BoatMovement__dzr(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_function__BoatMovement__dzr(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__fetch_function__BoatMovement__dzr(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_const_function__BoatMovement__dzr(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__assign_function__BoatMovement__dzr(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_function__BoatMovement__dzr(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__resize_function__BoatMovement__dzr(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_message_member_array[3] = {
  {
    "dx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__BoatMovement, dx),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__size_function__BoatMovement__dx,  // size() function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_const_function__BoatMovement__dx,  // get_const(index) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_function__BoatMovement__dx,  // get(index) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__fetch_function__BoatMovement__dx,  // fetch(index, &value) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__assign_function__BoatMovement__dx,  // assign(index, value) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__resize_function__BoatMovement__dx  // resize(index) function pointer
  },
  {
    "dy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__BoatMovement, dy),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__size_function__BoatMovement__dy,  // size() function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_const_function__BoatMovement__dy,  // get_const(index) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_function__BoatMovement__dy,  // get(index) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__fetch_function__BoatMovement__dy,  // fetch(index, &value) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__assign_function__BoatMovement__dy,  // assign(index, value) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__resize_function__BoatMovement__dy  // resize(index) function pointer
  },
  {
    "dzr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__BoatMovement, dzr),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__size_function__BoatMovement__dzr,  // size() function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_const_function__BoatMovement__dzr,  // get_const(index) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__get_function__BoatMovement__dzr,  // get(index) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__fetch_function__BoatMovement__dzr,  // fetch(index, &value) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__assign_function__BoatMovement__dzr,  // assign(index, value) function pointer
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__resize_function__BoatMovement__dzr  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_message_members = {
  "boat_interfaces__msg",  // message namespace
  "BoatMovement",  // message name
  3,  // number of fields
  sizeof(boat_interfaces__msg__BoatMovement),
  boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_message_member_array,  // message members
  boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_init_function,  // function to initialize message memory (memory has to be allocated)
  boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_message_type_support_handle = {
  0,
  &boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_boat_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, boat_interfaces, msg, BoatMovement)() {
  if (!boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_message_type_support_handle.typesupport_identifier) {
    boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &boat_interfaces__msg__BoatMovement__rosidl_typesupport_introspection_c__BoatMovement_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
