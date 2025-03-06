// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from boat_interfaces:msg/BuoyCoordinates.idl
// generated code does not contain a copyright notice
#include "boat_interfaces/msg/detail/buoy_coordinates__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "boat_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "boat_interfaces/msg/detail/buoy_coordinates__struct.h"
#include "boat_interfaces/msg/detail/buoy_coordinates__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // latitudes, longitudes
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // latitudes, longitudes
#include "rosidl_runtime_c/string.h"  // types
#include "rosidl_runtime_c/string_functions.h"  // types

// forward declare type support functions


using _BuoyCoordinates__ros_msg_type = boat_interfaces__msg__BuoyCoordinates;

static bool _BuoyCoordinates__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BuoyCoordinates__ros_msg_type * ros_message = static_cast<const _BuoyCoordinates__ros_msg_type *>(untyped_ros_message);
  // Field name: latitudes
  {
    size_t size = ros_message->latitudes.size;
    auto array_ptr = ros_message->latitudes.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: longitudes
  {
    size_t size = ros_message->longitudes.size;
    auto array_ptr = ros_message->longitudes.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: types
  {
    size_t size = ros_message->types.size;
    auto array_ptr = ros_message->types.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  return true;
}

static bool _BuoyCoordinates__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BuoyCoordinates__ros_msg_type * ros_message = static_cast<_BuoyCoordinates__ros_msg_type *>(untyped_ros_message);
  // Field name: latitudes
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->latitudes.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->latitudes);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->latitudes, size)) {
      fprintf(stderr, "failed to create array for field 'latitudes'");
      return false;
    }
    auto array_ptr = ros_message->latitudes.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: longitudes
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->longitudes.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->longitudes);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->longitudes, size)) {
      fprintf(stderr, "failed to create array for field 'longitudes'");
      return false;
    }
    auto array_ptr = ros_message->longitudes.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: types
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->types.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->types);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->types, size)) {
      fprintf(stderr, "failed to create array for field 'types'");
      return false;
    }
    auto array_ptr = ros_message->types.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'types'\n");
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_boat_interfaces
size_t get_serialized_size_boat_interfaces__msg__BuoyCoordinates(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BuoyCoordinates__ros_msg_type * ros_message = static_cast<const _BuoyCoordinates__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name latitudes
  {
    size_t array_size = ros_message->latitudes.size;
    auto array_ptr = ros_message->latitudes.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name longitudes
  {
    size_t array_size = ros_message->longitudes.size;
    auto array_ptr = ros_message->longitudes.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name types
  {
    size_t array_size = ros_message->types.size;
    auto array_ptr = ros_message->types.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BuoyCoordinates__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_boat_interfaces__msg__BuoyCoordinates(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_boat_interfaces
size_t max_serialized_size_boat_interfaces__msg__BuoyCoordinates(
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

  // member: latitudes
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
  // member: longitudes
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
  // member: types
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = boat_interfaces__msg__BuoyCoordinates;
    is_plain =
      (
      offsetof(DataType, types) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _BuoyCoordinates__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_boat_interfaces__msg__BuoyCoordinates(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_BuoyCoordinates = {
  "boat_interfaces::msg",
  "BuoyCoordinates",
  _BuoyCoordinates__cdr_serialize,
  _BuoyCoordinates__cdr_deserialize,
  _BuoyCoordinates__get_serialized_size,
  _BuoyCoordinates__max_serialized_size
};

static rosidl_message_type_support_t _BuoyCoordinates__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BuoyCoordinates,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, boat_interfaces, msg, BuoyCoordinates)() {
  return &_BuoyCoordinates__type_support;
}

#if defined(__cplusplus)
}
#endif
