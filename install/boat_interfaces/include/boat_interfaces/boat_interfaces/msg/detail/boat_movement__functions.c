// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from boat_interfaces:msg/BoatMovement.idl
// generated code does not contain a copyright notice
#include "boat_interfaces/msg/detail/boat_movement__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `dx`
// Member `dy`
// Member `dzr`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
boat_interfaces__msg__BoatMovement__init(boat_interfaces__msg__BoatMovement * msg)
{
  if (!msg) {
    return false;
  }
  // dx
  if (!rosidl_runtime_c__double__Sequence__init(&msg->dx, 0)) {
    boat_interfaces__msg__BoatMovement__fini(msg);
    return false;
  }
  // dy
  if (!rosidl_runtime_c__double__Sequence__init(&msg->dy, 0)) {
    boat_interfaces__msg__BoatMovement__fini(msg);
    return false;
  }
  // dzr
  if (!rosidl_runtime_c__double__Sequence__init(&msg->dzr, 0)) {
    boat_interfaces__msg__BoatMovement__fini(msg);
    return false;
  }
  return true;
}

void
boat_interfaces__msg__BoatMovement__fini(boat_interfaces__msg__BoatMovement * msg)
{
  if (!msg) {
    return;
  }
  // dx
  rosidl_runtime_c__double__Sequence__fini(&msg->dx);
  // dy
  rosidl_runtime_c__double__Sequence__fini(&msg->dy);
  // dzr
  rosidl_runtime_c__double__Sequence__fini(&msg->dzr);
}

bool
boat_interfaces__msg__BoatMovement__are_equal(const boat_interfaces__msg__BoatMovement * lhs, const boat_interfaces__msg__BoatMovement * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // dx
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->dx), &(rhs->dx)))
  {
    return false;
  }
  // dy
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->dy), &(rhs->dy)))
  {
    return false;
  }
  // dzr
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->dzr), &(rhs->dzr)))
  {
    return false;
  }
  return true;
}

bool
boat_interfaces__msg__BoatMovement__copy(
  const boat_interfaces__msg__BoatMovement * input,
  boat_interfaces__msg__BoatMovement * output)
{
  if (!input || !output) {
    return false;
  }
  // dx
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->dx), &(output->dx)))
  {
    return false;
  }
  // dy
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->dy), &(output->dy)))
  {
    return false;
  }
  // dzr
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->dzr), &(output->dzr)))
  {
    return false;
  }
  return true;
}

boat_interfaces__msg__BoatMovement *
boat_interfaces__msg__BoatMovement__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__BoatMovement * msg = (boat_interfaces__msg__BoatMovement *)allocator.allocate(sizeof(boat_interfaces__msg__BoatMovement), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(boat_interfaces__msg__BoatMovement));
  bool success = boat_interfaces__msg__BoatMovement__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
boat_interfaces__msg__BoatMovement__destroy(boat_interfaces__msg__BoatMovement * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    boat_interfaces__msg__BoatMovement__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
boat_interfaces__msg__BoatMovement__Sequence__init(boat_interfaces__msg__BoatMovement__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__BoatMovement * data = NULL;

  if (size) {
    data = (boat_interfaces__msg__BoatMovement *)allocator.zero_allocate(size, sizeof(boat_interfaces__msg__BoatMovement), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = boat_interfaces__msg__BoatMovement__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        boat_interfaces__msg__BoatMovement__fini(&data[i - 1]);
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
boat_interfaces__msg__BoatMovement__Sequence__fini(boat_interfaces__msg__BoatMovement__Sequence * array)
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
      boat_interfaces__msg__BoatMovement__fini(&array->data[i]);
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

boat_interfaces__msg__BoatMovement__Sequence *
boat_interfaces__msg__BoatMovement__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__BoatMovement__Sequence * array = (boat_interfaces__msg__BoatMovement__Sequence *)allocator.allocate(sizeof(boat_interfaces__msg__BoatMovement__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = boat_interfaces__msg__BoatMovement__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
boat_interfaces__msg__BoatMovement__Sequence__destroy(boat_interfaces__msg__BoatMovement__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    boat_interfaces__msg__BoatMovement__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
boat_interfaces__msg__BoatMovement__Sequence__are_equal(const boat_interfaces__msg__BoatMovement__Sequence * lhs, const boat_interfaces__msg__BoatMovement__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!boat_interfaces__msg__BoatMovement__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
boat_interfaces__msg__BoatMovement__Sequence__copy(
  const boat_interfaces__msg__BoatMovement__Sequence * input,
  boat_interfaces__msg__BoatMovement__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(boat_interfaces__msg__BoatMovement);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    boat_interfaces__msg__BoatMovement * data =
      (boat_interfaces__msg__BoatMovement *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!boat_interfaces__msg__BoatMovement__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          boat_interfaces__msg__BoatMovement__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!boat_interfaces__msg__BoatMovement__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
