// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavSat.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_sat__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sv_info`
#include "ublox_ubx_msgs/msg/detail/sat_info__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavSat__init(ublox_ubx_msgs__msg__UBXNavSat * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavSat__fini(msg);
    return false;
  }
  // itow
  // version
  // num_svs
  // sv_info
  if (!ublox_ubx_msgs__msg__SatInfo__Sequence__init(&msg->sv_info, 0)) {
    ublox_ubx_msgs__msg__UBXNavSat__fini(msg);
    return false;
  }
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavSat__fini(ublox_ubx_msgs__msg__UBXNavSat * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // itow
  // version
  // num_svs
  // sv_info
  ublox_ubx_msgs__msg__SatInfo__Sequence__fini(&msg->sv_info);
}

bool
ublox_ubx_msgs__msg__UBXNavSat__are_equal(const ublox_ubx_msgs__msg__UBXNavSat * lhs, const ublox_ubx_msgs__msg__UBXNavSat * rhs)
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
  // itow
  if (lhs->itow != rhs->itow) {
    return false;
  }
  // version
  if (lhs->version != rhs->version) {
    return false;
  }
  // num_svs
  if (lhs->num_svs != rhs->num_svs) {
    return false;
  }
  // sv_info
  if (!ublox_ubx_msgs__msg__SatInfo__Sequence__are_equal(
      &(lhs->sv_info), &(rhs->sv_info)))
  {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavSat__copy(
  const ublox_ubx_msgs__msg__UBXNavSat * input,
  ublox_ubx_msgs__msg__UBXNavSat * output)
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
  // itow
  output->itow = input->itow;
  // version
  output->version = input->version;
  // num_svs
  output->num_svs = input->num_svs;
  // sv_info
  if (!ublox_ubx_msgs__msg__SatInfo__Sequence__copy(
      &(input->sv_info), &(output->sv_info)))
  {
    return false;
  }
  return true;
}

ublox_ubx_msgs__msg__UBXNavSat *
ublox_ubx_msgs__msg__UBXNavSat__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavSat * msg = (ublox_ubx_msgs__msg__UBXNavSat *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavSat), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavSat));
  bool success = ublox_ubx_msgs__msg__UBXNavSat__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavSat__destroy(ublox_ubx_msgs__msg__UBXNavSat * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavSat__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavSat__Sequence__init(ublox_ubx_msgs__msg__UBXNavSat__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavSat * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavSat *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavSat), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavSat__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavSat__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavSat__Sequence__fini(ublox_ubx_msgs__msg__UBXNavSat__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavSat__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavSat__Sequence *
ublox_ubx_msgs__msg__UBXNavSat__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavSat__Sequence * array = (ublox_ubx_msgs__msg__UBXNavSat__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavSat__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavSat__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavSat__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavSat__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavSat__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavSat__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavSat__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavSat__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavSat__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavSat__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavSat__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavSat__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavSat);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavSat * data =
      (ublox_ubx_msgs__msg__UBXNavSat *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavSat__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavSat__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavSat__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
