// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavOdo.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_odo__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavOdo__init(ublox_ubx_msgs__msg__UBXNavOdo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavOdo__fini(msg);
    return false;
  }
  // version
  // itow
  // distance
  // total_distance
  // distance_std
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavOdo__fini(ublox_ubx_msgs__msg__UBXNavOdo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // version
  // itow
  // distance
  // total_distance
  // distance_std
}

bool
ublox_ubx_msgs__msg__UBXNavOdo__are_equal(const ublox_ubx_msgs__msg__UBXNavOdo * lhs, const ublox_ubx_msgs__msg__UBXNavOdo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // version
  if (lhs->version != rhs->version) {
    return false;
  }
  // itow
  if (lhs->itow != rhs->itow) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // total_distance
  if (lhs->total_distance != rhs->total_distance) {
    return false;
  }
  // distance_std
  if (lhs->distance_std != rhs->distance_std) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavOdo__copy(
  const ublox_ubx_msgs__msg__UBXNavOdo * input,
  ublox_ubx_msgs__msg__UBXNavOdo * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // version
  output->version = input->version;
  // itow
  output->itow = input->itow;
  // distance
  output->distance = input->distance;
  // total_distance
  output->total_distance = input->total_distance;
  // distance_std
  output->distance_std = input->distance_std;
  return true;
}

ublox_ubx_msgs__msg__UBXNavOdo *
ublox_ubx_msgs__msg__UBXNavOdo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavOdo * msg = (ublox_ubx_msgs__msg__UBXNavOdo *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavOdo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavOdo));
  bool success = ublox_ubx_msgs__msg__UBXNavOdo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavOdo__destroy(ublox_ubx_msgs__msg__UBXNavOdo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavOdo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavOdo__Sequence__init(ublox_ubx_msgs__msg__UBXNavOdo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavOdo * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavOdo *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavOdo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavOdo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavOdo__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavOdo__Sequence__fini(ublox_ubx_msgs__msg__UBXNavOdo__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavOdo__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavOdo__Sequence *
ublox_ubx_msgs__msg__UBXNavOdo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavOdo__Sequence * array = (ublox_ubx_msgs__msg__UBXNavOdo__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavOdo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavOdo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavOdo__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavOdo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavOdo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavOdo__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavOdo__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavOdo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavOdo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavOdo__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavOdo__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavOdo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavOdo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavOdo * data =
      (ublox_ubx_msgs__msg__UBXNavOdo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavOdo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavOdo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavOdo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
