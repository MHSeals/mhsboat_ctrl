// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from boat_interfaces:msg/BuoyMap.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BUOY_MAP__STRUCT_H_
#define BOAT_INTERFACES__MSG__DETAIL__BUOY_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'x'
// Member 'y'
// Member 'z'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'types'
// Member 'colors'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BuoyMap in the package boat_interfaces.
typedef struct boat_interfaces__msg__BuoyMap
{
  rosidl_runtime_c__double__Sequence x;
  rosidl_runtime_c__double__Sequence y;
  rosidl_runtime_c__double__Sequence z;
  rosidl_runtime_c__String__Sequence types;
  rosidl_runtime_c__String__Sequence colors;
} boat_interfaces__msg__BuoyMap;

// Struct for a sequence of boat_interfaces__msg__BuoyMap.
typedef struct boat_interfaces__msg__BuoyMap__Sequence
{
  boat_interfaces__msg__BuoyMap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} boat_interfaces__msg__BuoyMap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOAT_INTERFACES__MSG__DETAIL__BUOY_MAP__STRUCT_H_
