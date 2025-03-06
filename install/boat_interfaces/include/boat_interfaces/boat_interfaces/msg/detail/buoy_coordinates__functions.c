// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from boat_interfaces:msg/BuoyCoordinates.idl
// generated code does not contain a copyright notice
#include "boat_interfaces/msg/detail/buoy_coordinates__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `latitudes`
// Member `longitudes`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `types`
#include "rosidl_runtime_c/string_functions.h"

bool
boat_interfaces__msg__BuoyCoordinates__init(boat_interfaces__msg__BuoyCoordinates * msg)
{
  if (!msg) {
    return false;
  }
  // latitudes
  if (!rosidl_runtime_c__double__Sequence__init(&msg->latitudes, 0)) {
    boat_interfaces__msg__BuoyCoordinates__fini(msg);
    return false;
  }
  // longitudes
  if (!rosidl_runtime_c__double__Sequence__init(&msg->longitudes, 0)) {
    boat_interfaces__msg__BuoyCoordinates__fini(msg);
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__init(&msg->types, 0)) {
    boat_interfaces__msg__BuoyCoordinates__fini(msg);
    return false;
  }
  return true;
}

void
boat_interfaces__msg__BuoyCoordinates__fini(boat_interfaces__msg__BuoyCoordinates * msg)
{
  if (!msg) {
    return;
  }
  // latitudes
  rosidl_runtime_c__double__Sequence__fini(&msg->latitudes);
  // longitudes
  rosidl_runtime_c__double__Sequence__fini(&msg->longitudes);
  // types
  rosidl_runtime_c__String__Sequence__fini(&msg->types);
}

bool
boat_interfaces__msg__BuoyCoordinates__are_equal(const boat_interfaces__msg__BuoyCoordinates * lhs, const boat_interfaces__msg__BuoyCoordinates * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // latitudes
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->latitudes), &(rhs->latitudes)))
  {
    return false;
  }
  // longitudes
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->longitudes), &(rhs->longitudes)))
  {
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->types), &(rhs->types)))
  {
    return false;
  }
  return true;
}

bool
boat_interfaces__msg__BuoyCoordinates__copy(
  const boat_interfaces__msg__BuoyCoordinates * input,
  boat_interfaces__msg__BuoyCoordinates * output)
{
  if (!input || !output) {
    return false;
  }
  // latitudes
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->latitudes), &(output->latitudes)))
  {
    return false;
  }
  // longitudes
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->longitudes), &(output->longitudes)))
  {
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->types), &(output->types)))
  {
    return false;
  }
  return true;
}

boat_interfaces__msg__BuoyCoordinates *
boat_interfaces__msg__BuoyCoordinates__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__BuoyCoordinates * msg = (boat_interfaces__msg__BuoyCoordinates *)allocator.allocate(sizeof(boat_interfaces__msg__BuoyCoordinates), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(boat_interfaces__msg__BuoyCoordinates));
  bool success = boat_interfaces__msg__BuoyCoordinates__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
boat_interfaces__msg__BuoyCoordinates__destroy(boat_interfaces__msg__BuoyCoordinates * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    boat_interfaces__msg__BuoyCoordinates__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
boat_interfaces__msg__BuoyCoordinates__Sequence__init(boat_interfaces__msg__BuoyCoordinates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__BuoyCoordinates * data = NULL;

  if (size) {
    data = (boat_interfaces__msg__BuoyCoordinates *)allocator.zero_allocate(size, sizeof(boat_interfaces__msg__BuoyCoordinates), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = boat_interfaces__msg__BuoyCoordinates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        boat_interfaces__msg__BuoyCoordinates__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
boat_interfaces__msg__BuoyCoordinates__Sequence__fini(boat_interfaces__msg__BuoyCoordinates__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      boat_interfaces__msg__BuoyCoordinates__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

boat_interfaces__msg__BuoyCoordinates__Sequence *
boat_interfaces__msg__BuoyCoordinates__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__BuoyCoordinates__Sequence * array = (boat_interfaces__msg__BuoyCoordinates__Sequence *)allocator.allocate(sizeof(boat_interfaces__msg__BuoyCoordinates__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = boat_interfaces__msg__BuoyCoordinates__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
boat_interfaces__msg__BuoyCoordinates__Sequence__destroy(boat_interfaces__msg__BuoyCoordinates__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    boat_interfaces__msg__BuoyCoordinates__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
boat_interfaces__msg__BuoyCoordinates__Sequence__are_equal(const boat_interfaces__msg__BuoyCoordinates__Sequence * lhs, const boat_interfaces__msg__BuoyCoordinates__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!boat_interfaces__msg__BuoyCoordinates__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
boat_interfaces__msg__BuoyCoordinates__Sequence__copy(
  const boat_interfaces__msg__BuoyCoordinates__Sequence * input,
  boat_interfaces__msg__BuoyCoordinates__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(boat_interfaces__msg__BuoyCoordinates);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    boat_interfaces__msg__BuoyCoordinates * data =
      (boat_interfaces__msg__BuoyCoordinates *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!boat_interfaces__msg__BuoyCoordinates__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          boat_interfaces__msg__BuoyCoordinates__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!boat_interfaces__msg__BuoyCoordinates__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
