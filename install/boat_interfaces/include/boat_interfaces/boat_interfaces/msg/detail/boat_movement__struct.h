// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from boat_interfaces:msg/BoatMovement.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__STRUCT_H_
#define BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'dx'
// Member 'dy'
// Member 'dzr'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BoatMovement in the package boat_interfaces.
typedef struct boat_interfaces__msg__BoatMovement
{
  rosidl_runtime_c__double__Sequence dx;
  rosidl_runtime_c__double__Sequence dy;
  rosidl_runtime_c__double__Sequence dzr;
} boat_interfaces__msg__BoatMovement;

// Struct for a sequence of boat_interfaces__msg__BoatMovement.
typedef struct boat_interfaces__msg__BoatMovement__Sequence
{
  boat_interfaces__msg__BoatMovement * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} boat_interfaces__msg__BoatMovement__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOAT_INTERFACES__MSG__DETAIL__BOAT_MOVEMENT__STRUCT_H_
