// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from boat_interfaces:msg/AiOutput.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "boat_interfaces/msg/detail/ai_output__rosidl_typesupport_introspection_c.h"
#include "boat_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "boat_interfaces/msg/detail/ai_output__functions.h"
#include "boat_interfaces/msg/detail/ai_output__struct.h"


// Include directives for member types
// Member `types`
#include "rosidl_runtime_c/string_functions.h"
// Member `confidences`
// Member `lefts`
// Member `tops`
// Member `widths`
// Member `heights`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  boat_interfaces__msg__AiOutput__init(message_memory);
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_fini_function(void * message_memory)
{
  boat_interfaces__msg__AiOutput__fini(message_memory);
}

size_t boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__types(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__types(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__types(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__types(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__types(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__types(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__types(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__types(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__confidences(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__confidences(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__confidences(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__confidences(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__confidences(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__confidences(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__confidences(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__confidences(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__lefts(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__lefts(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__lefts(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__lefts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__lefts(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__lefts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__lefts(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__lefts(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__tops(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__tops(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__tops(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__tops(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__tops(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__tops(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__tops(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__tops(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__widths(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__widths(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__widths(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__widths(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__widths(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__widths(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__widths(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__widths(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__heights(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__heights(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__heights(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__heights(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__heights(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__heights(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__heights(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__heights(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_message_member_array[9] = {
  {
    "num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__AiOutput, num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "img_width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__AiOutput, img_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "img_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__AiOutput, img_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "types",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__AiOutput, types),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__types,  // size() function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__types,  // get_const(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__types,  // get(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__types,  // fetch(index, &value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__types,  // assign(index, value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__types  // resize(index) function pointer
  },
  {
    "confidences",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__AiOutput, confidences),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__confidences,  // size() function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__confidences,  // get_const(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__confidences,  // get(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__confidences,  // fetch(index, &value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__confidences,  // assign(index, value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__confidences  // resize(index) function pointer
  },
  {
    "lefts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__AiOutput, lefts),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__lefts,  // size() function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__lefts,  // get_const(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__lefts,  // get(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__lefts,  // fetch(index, &value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__lefts,  // assign(index, value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__lefts  // resize(index) function pointer
  },
  {
    "tops",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__AiOutput, tops),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__tops,  // size() function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__tops,  // get_const(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__tops,  // get(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__tops,  // fetch(index, &value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__tops,  // assign(index, value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__tops  // resize(index) function pointer
  },
  {
    "widths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__AiOutput, widths),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__widths,  // size() function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__widths,  // get_const(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__widths,  // get(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__widths,  // fetch(index, &value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__widths,  // assign(index, value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__widths  // resize(index) function pointer
  },
  {
    "heights",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(boat_interfaces__msg__AiOutput, heights),  // bytes offset in struct
    NULL,  // default value
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__size_function__AiOutput__heights,  // size() function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_const_function__AiOutput__heights,  // get_const(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__get_function__AiOutput__heights,  // get(index) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__fetch_function__AiOutput__heights,  // fetch(index, &value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__assign_function__AiOutput__heights,  // assign(index, value) function pointer
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__resize_function__AiOutput__heights  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_message_members = {
  "boat_interfaces__msg",  // message namespace
  "AiOutput",  // message name
  9,  // number of fields
  sizeof(boat_interfaces__msg__AiOutput),
  boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_message_member_array,  // message members
  boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_init_function,  // function to initialize message memory (memory has to be allocated)
  boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_message_type_support_handle = {
  0,
  &boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_boat_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, boat_interfaces, msg, AiOutput)() {
  if (!boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_message_type_support_handle.typesupport_identifier) {
    boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &boat_interfaces__msg__AiOutput__rosidl_typesupport_introspection_c__AiOutput_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
