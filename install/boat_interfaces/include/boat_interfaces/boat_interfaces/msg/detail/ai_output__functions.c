// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from boat_interfaces:msg/AiOutput.idl
// generated code does not contain a copyright notice
#include "boat_interfaces/msg/detail/ai_output__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `types`
#include "rosidl_runtime_c/string_functions.h"
// Member `confidences`
// Member `lefts`
// Member `tops`
// Member `widths`
// Member `heights`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
boat_interfaces__msg__AiOutput__init(boat_interfaces__msg__AiOutput * msg)
{
  if (!msg) {
    return false;
  }
  // num
  // img_width
  // img_height
  // types
  if (!rosidl_runtime_c__String__Sequence__init(&msg->types, 0)) {
    boat_interfaces__msg__AiOutput__fini(msg);
    return false;
  }
  // confidences
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->confidences, 0)) {
    boat_interfaces__msg__AiOutput__fini(msg);
    return false;
  }
  // lefts
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->lefts, 0)) {
    boat_interfaces__msg__AiOutput__fini(msg);
    return false;
  }
  // tops
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->tops, 0)) {
    boat_interfaces__msg__AiOutput__fini(msg);
    return false;
  }
  // widths
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->widths, 0)) {
    boat_interfaces__msg__AiOutput__fini(msg);
    return false;
  }
  // heights
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->heights, 0)) {
    boat_interfaces__msg__AiOutput__fini(msg);
    return false;
  }
  return true;
}

void
boat_interfaces__msg__AiOutput__fini(boat_interfaces__msg__AiOutput * msg)
{
  if (!msg) {
    return;
  }
  // num
  // img_width
  // img_height
  // types
  rosidl_runtime_c__String__Sequence__fini(&msg->types);
  // confidences
  rosidl_runtime_c__int32__Sequence__fini(&msg->confidences);
  // lefts
  rosidl_runtime_c__int32__Sequence__fini(&msg->lefts);
  // tops
  rosidl_runtime_c__int32__Sequence__fini(&msg->tops);
  // widths
  rosidl_runtime_c__int32__Sequence__fini(&msg->widths);
  // heights
  rosidl_runtime_c__int32__Sequence__fini(&msg->heights);
}

bool
boat_interfaces__msg__AiOutput__are_equal(const boat_interfaces__msg__AiOutput * lhs, const boat_interfaces__msg__AiOutput * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // num
  if (lhs->num != rhs->num) {
    return false;
  }
  // img_width
  if (lhs->img_width != rhs->img_width) {
    return false;
  }
  // img_height
  if (lhs->img_height != rhs->img_height) {
    return false;
  }
  // types
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->types), &(rhs->types)))
  {
    return false;
  }
  // confidences
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->confidences), &(rhs->confidences)))
  {
    return false;
  }
  // lefts
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->lefts), &(rhs->lefts)))
  {
    return false;
  }
  // tops
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->tops), &(rhs->tops)))
  {
    return false;
  }
  // widths
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->widths), &(rhs->widths)))
  {
    return false;
  }
  // heights
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->heights), &(rhs->heights)))
  {
    return false;
  }
  return true;
}

bool
boat_interfaces__msg__AiOutput__copy(
  const boat_interfaces__msg__AiOutput * input,
  boat_interfaces__msg__AiOutput * output)
{
  if (!input || !output) {
    return false;
  }
  // num
  output->num = input->num;
  // img_width
  output->img_width = input->img_width;
  // img_height
  output->img_height = input->img_height;
  // types
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->types), &(output->types)))
  {
    return false;
  }
  // confidences
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->confidences), &(output->confidences)))
  {
    return false;
  }
  // lefts
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->lefts), &(output->lefts)))
  {
    return false;
  }
  // tops
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->tops), &(output->tops)))
  {
    return false;
  }
  // widths
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->widths), &(output->widths)))
  {
    return false;
  }
  // heights
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->heights), &(output->heights)))
  {
    return false;
  }
  return true;
}

boat_interfaces__msg__AiOutput *
boat_interfaces__msg__AiOutput__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__AiOutput * msg = (boat_interfaces__msg__AiOutput *)allocator.allocate(sizeof(boat_interfaces__msg__AiOutput), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(boat_interfaces__msg__AiOutput));
  bool success = boat_interfaces__msg__AiOutput__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
boat_interfaces__msg__AiOutput__destroy(boat_interfaces__msg__AiOutput * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    boat_interfaces__msg__AiOutput__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
boat_interfaces__msg__AiOutput__Sequence__init(boat_interfaces__msg__AiOutput__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__AiOutput * data = NULL;

  if (size) {
    data = (boat_interfaces__msg__AiOutput *)allocator.zero_allocate(size, sizeof(boat_interfaces__msg__AiOutput), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = boat_interfaces__msg__AiOutput__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        boat_interfaces__msg__AiOutput__fini(&data[i - 1]);
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
boat_interfaces__msg__AiOutput__Sequence__fini(boat_interfaces__msg__AiOutput__Sequence * array)
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
      boat_interfaces__msg__AiOutput__fini(&array->data[i]);
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

boat_interfaces__msg__AiOutput__Sequence *
boat_interfaces__msg__AiOutput__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  boat_interfaces__msg__AiOutput__Sequence * array = (boat_interfaces__msg__AiOutput__Sequence *)allocator.allocate(sizeof(boat_interfaces__msg__AiOutput__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = boat_interfaces__msg__AiOutput__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
boat_interfaces__msg__AiOutput__Sequence__destroy(boat_interfaces__msg__AiOutput__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    boat_interfaces__msg__AiOutput__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
boat_interfaces__msg__AiOutput__Sequence__are_equal(const boat_interfaces__msg__AiOutput__Sequence * lhs, const boat_interfaces__msg__AiOutput__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!boat_interfaces__msg__AiOutput__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
boat_interfaces__msg__AiOutput__Sequence__copy(
  const boat_interfaces__msg__AiOutput__Sequence * input,
  boat_interfaces__msg__AiOutput__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(boat_interfaces__msg__AiOutput);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    boat_interfaces__msg__AiOutput * data =
      (boat_interfaces__msg__AiOutput *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!boat_interfaces__msg__AiOutput__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          boat_interfaces__msg__AiOutput__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!boat_interfaces__msg__AiOutput__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
