// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from iw_interfaces:msg/Area.idl
// generated code does not contain a copyright notice
#include "iw_interfaces/msg/detail/area__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `label`
#include "rosidl_runtime_c/string_functions.h"
// Member `vertices`
#include "iw_interfaces/msg/detail/plane_coordinate__functions.h"

bool
iw_interfaces__msg__Area__init(iw_interfaces__msg__Area * msg)
{
  if (!msg) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    iw_interfaces__msg__Area__fini(msg);
    return false;
  }
  // vertices
  if (!iw_interfaces__msg__PlaneCoordinate__Sequence__init(&msg->vertices, 0)) {
    iw_interfaces__msg__Area__fini(msg);
    return false;
  }
  return true;
}

void
iw_interfaces__msg__Area__fini(iw_interfaces__msg__Area * msg)
{
  if (!msg) {
    return;
  }
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // vertices
  iw_interfaces__msg__PlaneCoordinate__Sequence__fini(&msg->vertices);
}

bool
iw_interfaces__msg__Area__are_equal(const iw_interfaces__msg__Area * lhs, const iw_interfaces__msg__Area * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // vertices
  if (!iw_interfaces__msg__PlaneCoordinate__Sequence__are_equal(
      &(lhs->vertices), &(rhs->vertices)))
  {
    return false;
  }
  return true;
}

bool
iw_interfaces__msg__Area__copy(
  const iw_interfaces__msg__Area * input,
  iw_interfaces__msg__Area * output)
{
  if (!input || !output) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // vertices
  if (!iw_interfaces__msg__PlaneCoordinate__Sequence__copy(
      &(input->vertices), &(output->vertices)))
  {
    return false;
  }
  return true;
}

iw_interfaces__msg__Area *
iw_interfaces__msg__Area__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  iw_interfaces__msg__Area * msg = (iw_interfaces__msg__Area *)allocator.allocate(sizeof(iw_interfaces__msg__Area), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(iw_interfaces__msg__Area));
  bool success = iw_interfaces__msg__Area__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
iw_interfaces__msg__Area__destroy(iw_interfaces__msg__Area * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    iw_interfaces__msg__Area__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
iw_interfaces__msg__Area__Sequence__init(iw_interfaces__msg__Area__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  iw_interfaces__msg__Area * data = NULL;

  if (size) {
    data = (iw_interfaces__msg__Area *)allocator.zero_allocate(size, sizeof(iw_interfaces__msg__Area), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = iw_interfaces__msg__Area__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        iw_interfaces__msg__Area__fini(&data[i - 1]);
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
iw_interfaces__msg__Area__Sequence__fini(iw_interfaces__msg__Area__Sequence * array)
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
      iw_interfaces__msg__Area__fini(&array->data[i]);
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

iw_interfaces__msg__Area__Sequence *
iw_interfaces__msg__Area__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  iw_interfaces__msg__Area__Sequence * array = (iw_interfaces__msg__Area__Sequence *)allocator.allocate(sizeof(iw_interfaces__msg__Area__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = iw_interfaces__msg__Area__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
iw_interfaces__msg__Area__Sequence__destroy(iw_interfaces__msg__Area__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    iw_interfaces__msg__Area__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
iw_interfaces__msg__Area__Sequence__are_equal(const iw_interfaces__msg__Area__Sequence * lhs, const iw_interfaces__msg__Area__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!iw_interfaces__msg__Area__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
iw_interfaces__msg__Area__Sequence__copy(
  const iw_interfaces__msg__Area__Sequence * input,
  iw_interfaces__msg__Area__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(iw_interfaces__msg__Area);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    iw_interfaces__msg__Area * data =
      (iw_interfaces__msg__Area *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!iw_interfaces__msg__Area__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          iw_interfaces__msg__Area__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!iw_interfaces__msg__Area__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
