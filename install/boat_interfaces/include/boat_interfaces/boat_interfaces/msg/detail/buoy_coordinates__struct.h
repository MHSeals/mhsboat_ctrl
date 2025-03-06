// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from boat_interfaces:msg/BuoyCoordinates.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__STRUCT_H_
#define BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'latitudes'
// Member 'longitudes'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'types'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BuoyCoordinates in the package boat_interfaces.
typedef struct boat_interfaces__msg__BuoyCoordinates
{
  rosidl_runtime_c__double__Sequence latitudes;
  rosidl_runtime_c__double__Sequence longitudes;
  rosidl_runtime_c__String__Sequence types;
} boat_interfaces__msg__BuoyCoordinates;

// Struct for a sequence of boat_interfaces__msg__BuoyCoordinates.
typedef struct boat_interfaces__msg__BuoyCoordinates__Sequence
{
  boat_interfaces__msg__BuoyCoordinates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} boat_interfaces__msg__BuoyCoordinates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOAT_INTERFACES__MSG__DETAIL__BUOY_COORDINATES__STRUCT_H_
