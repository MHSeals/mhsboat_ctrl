// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from boat_interfaces:msg/BoatMovement.idl
// generated code does not contain a copyright notice
#include "boat_interfaces/msg/detail/boat_movement__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "boat_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "boat_interfaces/msg/detail/boat_movement__struct.h"
#include "boat_interfaces/msg/detail/boat_movement__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // dx, dy, dzr
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // dx, dy, dzr

// forward declare type support functions


using _BoatMovement__ros_msg_type = boat_interfaces__msg__BoatMovement;

static bool _BoatMovement__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BoatMovement__ros_msg_type * ros_message = static_cast<const _BoatMovement__ros_msg_type *>(untyped_ros_message);
  // Field name: dx
  {
    size_t size = ros_message->dx.size;
    auto array_ptr = ros_message->dx.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: dy
  {
    size_t size = ros_message->dy.size;
    auto array_ptr = ros_message->dy.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: dzr
  {
    size_t size = ros_message->dzr.size;
    auto array_ptr = ros_message->dzr.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _BoatMovement__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BoatMovement__ros_msg_type * ros_message = static_cast<_BoatMovement__ros_msg_type *>(untyped_ros_message);
  // Field name: dx
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->dx.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->dx);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->dx, size)) {
      fprintf(stderr, "failed to create array for field 'dx'");
      return false;
    }
    auto array_ptr = ros_message->dx.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: dy
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->dy.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->dy);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->dy, size)) {
      fprintf(stderr, "failed to create array for field 'dy'");
      return false;
    }
    auto array_ptr = ros_message->dy.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: dzr
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->dzr.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->dzr);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->dzr, size)) {
      fprintf(stderr, "failed to create array for field 'dzr'");
      return false;
    }
    auto array_ptr = ros_message->dzr.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_boat_interfaces
size_t get_serialized_size_boat_interfaces__msg__BoatMovement(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BoatMovement__ros_msg_type * ros_message = static_cast<const _BoatMovement__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name dx
  {
    size_t array_size = ros_message->dx.size;
    auto array_ptr = ros_message->dx.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dy
  {
    size_t array_size = ros_message->dy.size;
    auto array_ptr = ros_message->dy.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dzr
  {
    size_t array_size = ros_message->dzr.size;
    auto array_ptr = ros_message->dzr.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BoatMovement__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_boat_interfaces__msg__BoatMovement(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_boat_interfaces
size_t max_serialized_size_boat_interfaces__msg__BoatMovement(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: dx
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: dy
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: dzr
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = boat_interfaces__msg__BoatMovement;
    is_plain =
      (
      offsetof(DataType, dzr) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _BoatMovement__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_boat_interfaces__msg__BoatMovement(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_BoatMovement = {
  "boat_interfaces::msg",
  "BoatMovement",
  _BoatMovement__cdr_serialize,
  _BoatMovement__cdr_deserialize,
  _BoatMovement__get_serialized_size,
  _BoatMovement__max_serialized_size
};

static rosidl_message_type_support_t _BoatMovement__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BoatMovement,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, boat_interfaces, msg, BoatMovement)() {
  return &_BoatMovement__type_support;
}

#if defined(__cplusplus)
}
#endif
