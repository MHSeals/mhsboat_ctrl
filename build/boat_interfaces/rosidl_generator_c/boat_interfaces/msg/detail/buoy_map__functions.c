// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from boat_interfaces:msg/BuoyMap.idl
// generated code does not contain a copyright notice
#include "boat_interfaces/msg/detail/buoy_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `x`
// Member `y`
// Member `z`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `types`
// Member `colors`
#include "rosidl_runtime_c/string_functions.h"

bool
boat_interfaces__msg__BuoyMap__init(boat_interfaces__msg__BuoyMap * msg)
{
  if (!msg) {
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->x, 0)) {
    boat_interfaces__msg__BuoyMap__fini(msg);
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->y, 0)) {
    boat_interfaces__msg__BuoyMap__fini(msg);
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__init(&msg->z, 0)) {
    boat_interfaces__msg__BuoyMap__fini(msg);
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__init(&msg->types, 0)) {
    boat_interfaces__msg__BuoyMap__fini(msg);
    return false;
  }
  // colors
  if (!rosidl_runtime_c__String__Sequence__init(&msg->colors, 0)) {
    boat_interfaces__msg__BuoyMap__fini(msg);
    return false;
  }
  return true;
}

void
boat_interfaces__msg__BuoyMap__fini(boat_interfaces__msg__BuoyMap * msg)
{
  if (!msg) {
    return;
  }
  // x
  rosidl_runtime_c__double__Sequence__fini(&msg->x);
  // y
  rosidl_runtime_c__double__Sequence__fini(&msg->y);
  // z
  rosidl_runtime_c__double__Sequence__fini(&msg->z);
  // types
  rosidl_runtime_c__String__Sequence__fini(&msg->types);
  // colors
  rosidl_runtime_c__String__Sequence__fini(&msg->colors);
}

bool
boat_interfaces__msg__BuoyMap__are_equal(const boat_interfaces__msg__BuoyMap * lhs, const boat_interfaces__msg__BuoyMap * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->x), &(rhs->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->y), &(rhs->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->z), &(rhs->z)))
  {
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->types), &(rhs->types)))
  {
    return false;
  }
  // colors
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->colors), &(rhs->colors)))
  {
    return false;
  }
  return true;
}

bool
boat_interfaces__msg__BuoyMap__copy(
  const boat_interfaces__msg__BuoyMap * input,
  boat_interfaces__msg__BuoyMap * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->x), &(output->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->y), &(output->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->z), &(output->z)))
  {
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->types), &(output->types)))
  {
    return false;
  }
  // colors
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->colors), &(output->colors)))
  {
    return false;
  }
  return true;
}

boat_interfaces__msg__BuoyMap *
boat_interfaces__msg__BuoyMap__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__BuoyMap * msg = (boat_interfaces__msg__BuoyMap *)allocator.allocate(sizeof(boat_interfaces__msg__BuoyMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(boat_interfaces__msg__BuoyMap));
  bool success = boat_interfaces__msg__BuoyMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
boat_interfaces__msg__BuoyMap__destroy(boat_interfaces__msg__BuoyMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    boat_interfaces__msg__BuoyMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
boat_interfaces__msg__BuoyMap__Sequence__init(boat_interfaces__msg__BuoyMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__BuoyMap * data = NULL;

  if (size) {
    data = (boat_interfaces__msg__BuoyMap *)allocator.zero_allocate(size, sizeof(boat_interfaces__msg__BuoyMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = boat_interfaces__msg__BuoyMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        boat_interfaces__msg__BuoyMap__fini(&data[i - 1]);
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
boat_interfaces__msg__BuoyMap__Sequence__fini(boat_interfaces__msg__BuoyMap__Sequence * array)
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
      boat_interfaces__msg__BuoyMap__fini(&array->data[i]);
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

boat_interfaces__msg__BuoyMap__Sequence *
boat_interfaces__msg__BuoyMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__BuoyMap__Sequence * array = (boat_interfaces__msg__BuoyMap__Sequence *)allocator.allocate(sizeof(boat_interfaces__msg__BuoyMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = boat_interfaces__msg__BuoyMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
boat_interfaces__msg__BuoyMap__Sequence__destroy(boat_interfaces__msg__BuoyMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    boat_interfaces__msg__BuoyMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
boat_interfaces__msg__BuoyMap__Sequence__are_equal(const boat_interfaces__msg__BuoyMap__Sequence * lhs, const boat_interfaces__msg__BuoyMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!boat_interfaces__msg__BuoyMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
boat_interfaces__msg__BuoyMap__Sequence__copy(
  const boat_interfaces__msg__BuoyMap__Sequence * input,
  boat_interfaces__msg__BuoyMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(boat_interfaces__msg__BuoyMap);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    boat_interfaces__msg__BuoyMap * data =
      (boat_interfaces__msg__BuoyMap *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!boat_interfaces__msg__BuoyMap__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          boat_interfaces__msg__BuoyMap__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!boat_interfaces__msg__BuoyMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
