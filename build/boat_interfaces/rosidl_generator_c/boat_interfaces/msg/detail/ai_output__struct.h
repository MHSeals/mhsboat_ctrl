// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from boat_interfaces:msg/AiOutput.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__STRUCT_H_
#define BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'types'
#include "rosidl_runtime_c/string.h"
// Member 'confidences'
// Member 'lefts'
// Member 'tops'
// Member 'widths'
// Member 'heights'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/AiOutput in the package boat_interfaces.
typedef struct boat_interfaces__msg__AiOutput
{
  int32_t num;
  int32_t img_width;
  int32_t img_height;
  rosidl_runtime_c__String__Sequence types;
  rosidl_runtime_c__int32__Sequence confidences;
  rosidl_runtime_c__int32__Sequence lefts;
  rosidl_runtime_c__int32__Sequence tops;
  rosidl_runtime_c__int32__Sequence widths;
  rosidl_runtime_c__int32__Sequence heights;
} boat_interfaces__msg__AiOutput;

// Struct for a sequence of boat_interfaces__msg__AiOutput.
typedef struct boat_interfaces__msg__AiOutput__Sequence
{
  boat_interfaces__msg__AiOutput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} boat_interfaces__msg__AiOutput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__STRUCT_H_
