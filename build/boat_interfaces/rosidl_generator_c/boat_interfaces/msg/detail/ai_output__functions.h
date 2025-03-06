// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from boat_interfaces:msg/AiOutput.idl
// generated code does not contain a copyright notice

#ifndef BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__FUNCTIONS_H_
#define BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "boat_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "boat_interfaces/msg/detail/ai_output__struct.h"

/// Initialize msg/AiOutput message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * boat_interfaces__msg__AiOutput
 * )) before or use
 * boat_interfaces__msg__AiOutput__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
bool
boat_interfaces__msg__AiOutput__init(boat_interfaces__msg__AiOutput * msg);

/// Finalize msg/AiOutput message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
void
boat_interfaces__msg__AiOutput__fini(boat_interfaces__msg__AiOutput * msg);

/// Create msg/AiOutput message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * boat_interfaces__msg__AiOutput__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
boat_interfaces__msg__AiOutput *
boat_interfaces__msg__AiOutput__create();

/// Destroy msg/AiOutput message.
/**
 * It calls
 * boat_interfaces__msg__AiOutput__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
void
boat_interfaces__msg__AiOutput__destroy(boat_interfaces__msg__AiOutput * msg);

/// Check for msg/AiOutput message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
bool
boat_interfaces__msg__AiOutput__are_equal(const boat_interfaces__msg__AiOutput * lhs, const boat_interfaces__msg__AiOutput * rhs);

/// Copy a msg/AiOutput message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
bool
boat_interfaces__msg__AiOutput__copy(
  const boat_interfaces__msg__AiOutput * input,
  boat_interfaces__msg__AiOutput * output);

/// Initialize array of msg/AiOutput messages.
/**
 * It allocates the memory for the number of elements and calls
 * boat_interfaces__msg__AiOutput__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
bool
boat_interfaces__msg__AiOutput__Sequence__init(boat_interfaces__msg__AiOutput__Sequence * array, size_t size);

/// Finalize array of msg/AiOutput messages.
/**
 * It calls
 * boat_interfaces__msg__AiOutput__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
void
boat_interfaces__msg__AiOutput__Sequence__fini(boat_interfaces__msg__AiOutput__Sequence * array);

/// Create array of msg/AiOutput messages.
/**
 * It allocates the memory for the array and calls
 * boat_interfaces__msg__AiOutput__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
boat_interfaces__msg__AiOutput__Sequence *
boat_interfaces__msg__AiOutput__Sequence__create(size_t size);

/// Destroy array of msg/AiOutput messages.
/**
 * It calls
 * boat_interfaces__msg__AiOutput__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
void
boat_interfaces__msg__AiOutput__Sequence__destroy(boat_interfaces__msg__AiOutput__Sequence * array);

/// Check for msg/AiOutput message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
bool
boat_interfaces__msg__AiOutput__Sequence__are_equal(const boat_interfaces__msg__AiOutput__Sequence * lhs, const boat_interfaces__msg__AiOutput__Sequence * rhs);

/// Copy an array of msg/AiOutput messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_boat_interfaces
bool
boat_interfaces__msg__AiOutput__Sequence__copy(
  const boat_interfaces__msg__AiOutput__Sequence * input,
  boat_interfaces__msg__AiOutput__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BOAT_INTERFACES__MSG__DETAIL__AI_OUTPUT__FUNCTIONS_H_
