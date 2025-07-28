// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/SBASStatusFlags.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sbas_status_flags__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ublox_ubx_msgs__msg__SBASStatusFlags__init(ublox_ubx_msgs__msg__SBASStatusFlags * msg)
{
  if (!msg) {
    return false;
  }
  // integrity_used
  return true;
}

void
ublox_ubx_msgs__msg__SBASStatusFlags__fini(ublox_ubx_msgs__msg__SBASStatusFlags * msg)
{
  if (!msg) {
    return;
  }
  // integrity_used
}

bool
ublox_ubx_msgs__msg__SBASStatusFlags__are_equal(const ublox_ubx_msgs__msg__SBASStatusFlags * lhs, const ublox_ubx_msgs__msg__SBASStatusFlags * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // integrity_used
  if (lhs->integrity_used != rhs->integrity_used) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SBASStatusFlags__copy(
  const ublox_ubx_msgs__msg__SBASStatusFlags * input,
  ublox_ubx_msgs__msg__SBASStatusFlags * output)
{
  if (!input || !output) {
    return false;
  }
  // integrity_used
  output->integrity_used = input->integrity_used;
  return true;
}

ublox_ubx_msgs__msg__SBASStatusFlags *
ublox_ubx_msgs__msg__SBASStatusFlags__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SBASStatusFlags * msg = (ublox_ubx_msgs__msg__SBASStatusFlags *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SBASStatusFlags), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__SBASStatusFlags));
  bool success = ublox_ubx_msgs__msg__SBASStatusFlags__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__SBASStatusFlags__destroy(ublox_ubx_msgs__msg__SBASStatusFlags * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__SBASStatusFlags__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__SBASStatusFlags__Sequence__init(ublox_ubx_msgs__msg__SBASStatusFlags__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SBASStatusFlags * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__SBASStatusFlags *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__SBASStatusFlags), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__SBASStatusFlags__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__SBASStatusFlags__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__SBASStatusFlags__Sequence__fini(ublox_ubx_msgs__msg__SBASStatusFlags__Sequence * array)
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
      ublox_ubx_msgs__msg__SBASStatusFlags__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__SBASStatusFlags__Sequence *
ublox_ubx_msgs__msg__SBASStatusFlags__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SBASStatusFlags__Sequence * array = (ublox_ubx_msgs__msg__SBASStatusFlags__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SBASStatusFlags__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__SBASStatusFlags__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__SBASStatusFlags__Sequence__destroy(ublox_ubx_msgs__msg__SBASStatusFlags__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__SBASStatusFlags__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__SBASStatusFlags__Sequence__are_equal(const ublox_ubx_msgs__msg__SBASStatusFlags__Sequence * lhs, const ublox_ubx_msgs__msg__SBASStatusFlags__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__SBASStatusFlags__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SBASStatusFlags__Sequence__copy(
  const ublox_ubx_msgs__msg__SBASStatusFlags__Sequence * input,
  ublox_ubx_msgs__msg__SBASStatusFlags__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__SBASStatusFlags);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__SBASStatusFlags * data =
      (ublox_ubx_msgs__msg__SBASStatusFlags *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__SBASStatusFlags__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__SBASStatusFlags__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__SBASStatusFlags__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
