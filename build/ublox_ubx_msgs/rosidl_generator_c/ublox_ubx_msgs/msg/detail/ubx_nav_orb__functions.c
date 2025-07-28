// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavOrb.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_orb__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sv_info`
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavOrb__init(ublox_ubx_msgs__msg__UBXNavOrb * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavOrb__fini(msg);
    return false;
  }
  // itow
  // version
  // num_sv
  // reserved_0
  // sv_info
  if (!ublox_ubx_msgs__msg__OrbSVInfo__Sequence__init(&msg->sv_info, 0)) {
    ublox_ubx_msgs__msg__UBXNavOrb__fini(msg);
    return false;
  }
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavOrb__fini(ublox_ubx_msgs__msg__UBXNavOrb * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // itow
  // version
  // num_sv
  // reserved_0
  // sv_info
  ublox_ubx_msgs__msg__OrbSVInfo__Sequence__fini(&msg->sv_info);
}

bool
ublox_ubx_msgs__msg__UBXNavOrb__are_equal(const ublox_ubx_msgs__msg__UBXNavOrb * lhs, const ublox_ubx_msgs__msg__UBXNavOrb * rhs)
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
  // num_sv
  if (lhs->num_sv != rhs->num_sv) {
    return false;
  }
  // reserved_0
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->reserved_0[i] != rhs->reserved_0[i]) {
      return false;
    }
  }
  // sv_info
  if (!ublox_ubx_msgs__msg__OrbSVInfo__Sequence__are_equal(
      &(lhs->sv_info), &(rhs->sv_info)))
  {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavOrb__copy(
  const ublox_ubx_msgs__msg__UBXNavOrb * input,
  ublox_ubx_msgs__msg__UBXNavOrb * output)
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
  // num_sv
  output->num_sv = input->num_sv;
  // reserved_0
  for (size_t i = 0; i < 2; ++i) {
    output->reserved_0[i] = input->reserved_0[i];
  }
  // sv_info
  if (!ublox_ubx_msgs__msg__OrbSVInfo__Sequence__copy(
      &(input->sv_info), &(output->sv_info)))
  {
    return false;
  }
  return true;
}

ublox_ubx_msgs__msg__UBXNavOrb *
ublox_ubx_msgs__msg__UBXNavOrb__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavOrb * msg = (ublox_ubx_msgs__msg__UBXNavOrb *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavOrb), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavOrb));
  bool success = ublox_ubx_msgs__msg__UBXNavOrb__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavOrb__destroy(ublox_ubx_msgs__msg__UBXNavOrb * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavOrb__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavOrb__Sequence__init(ublox_ubx_msgs__msg__UBXNavOrb__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavOrb * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavOrb *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavOrb), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavOrb__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavOrb__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavOrb__Sequence__fini(ublox_ubx_msgs__msg__UBXNavOrb__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavOrb__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavOrb__Sequence *
ublox_ubx_msgs__msg__UBXNavOrb__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavOrb__Sequence * array = (ublox_ubx_msgs__msg__UBXNavOrb__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavOrb__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavOrb__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavOrb__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavOrb__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavOrb__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavOrb__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavOrb__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavOrb__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavOrb__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavOrb__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavOrb__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavOrb__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavOrb);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavOrb * data =
      (ublox_ubx_msgs__msg__UBXNavOrb *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavOrb__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavOrb__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavOrb__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
