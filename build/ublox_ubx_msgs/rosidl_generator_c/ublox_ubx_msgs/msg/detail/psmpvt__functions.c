// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/PSMPVT.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/psmpvt__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ublox_ubx_msgs__msg__PSMPVT__init(ublox_ubx_msgs__msg__PSMPVT * msg)
{
  if (!msg) {
    return false;
  }
  // state
  return true;
}

void
ublox_ubx_msgs__msg__PSMPVT__fini(ublox_ubx_msgs__msg__PSMPVT * msg)
{
  if (!msg) {
    return;
  }
  // state
}

bool
ublox_ubx_msgs__msg__PSMPVT__are_equal(const ublox_ubx_msgs__msg__PSMPVT * lhs, const ublox_ubx_msgs__msg__PSMPVT * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__PSMPVT__copy(
  const ublox_ubx_msgs__msg__PSMPVT * input,
  ublox_ubx_msgs__msg__PSMPVT * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  output->state = input->state;
  return true;
}

ublox_ubx_msgs__msg__PSMPVT *
ublox_ubx_msgs__msg__PSMPVT__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__PSMPVT * msg = (ublox_ubx_msgs__msg__PSMPVT *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__PSMPVT), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__PSMPVT));
  bool success = ublox_ubx_msgs__msg__PSMPVT__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__PSMPVT__destroy(ublox_ubx_msgs__msg__PSMPVT * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__PSMPVT__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__PSMPVT__Sequence__init(ublox_ubx_msgs__msg__PSMPVT__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__PSMPVT * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__PSMPVT *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__PSMPVT), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__PSMPVT__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__PSMPVT__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__PSMPVT__Sequence__fini(ublox_ubx_msgs__msg__PSMPVT__Sequence * array)
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
      ublox_ubx_msgs__msg__PSMPVT__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__PSMPVT__Sequence *
ublox_ubx_msgs__msg__PSMPVT__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__PSMPVT__Sequence * array = (ublox_ubx_msgs__msg__PSMPVT__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__PSMPVT__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__PSMPVT__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__PSMPVT__Sequence__destroy(ublox_ubx_msgs__msg__PSMPVT__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__PSMPVT__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__PSMPVT__Sequence__are_equal(const ublox_ubx_msgs__msg__PSMPVT__Sequence * lhs, const ublox_ubx_msgs__msg__PSMPVT__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__PSMPVT__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__PSMPVT__Sequence__copy(
  const ublox_ubx_msgs__msg__PSMPVT__Sequence * input,
  ublox_ubx_msgs__msg__PSMPVT__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__PSMPVT);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__PSMPVT * data =
      (ublox_ubx_msgs__msg__PSMPVT *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__PSMPVT__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__PSMPVT__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__PSMPVT__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
