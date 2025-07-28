// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/OrbSVInfo.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `sv_flag`
#include "ublox_ubx_msgs/msg/detail/orb_sv_flag__functions.h"
// Member `eph`
#include "ublox_ubx_msgs/msg/detail/orb_eph_info__functions.h"
// Member `alm`
#include "ublox_ubx_msgs/msg/detail/orb_alm_info__functions.h"
// Member `other_orb`
#include "ublox_ubx_msgs/msg/detail/other_orb_info__functions.h"

bool
ublox_ubx_msgs__msg__OrbSVInfo__init(ublox_ubx_msgs__msg__OrbSVInfo * msg)
{
  if (!msg) {
    return false;
  }
  // gnss_id
  // sv_id
  // sv_flag
  if (!ublox_ubx_msgs__msg__OrbSVFlag__init(&msg->sv_flag)) {
    ublox_ubx_msgs__msg__OrbSVInfo__fini(msg);
    return false;
  }
  // eph
  if (!ublox_ubx_msgs__msg__OrbEphInfo__init(&msg->eph)) {
    ublox_ubx_msgs__msg__OrbSVInfo__fini(msg);
    return false;
  }
  // alm
  if (!ublox_ubx_msgs__msg__OrbAlmInfo__init(&msg->alm)) {
    ublox_ubx_msgs__msg__OrbSVInfo__fini(msg);
    return false;
  }
  // other_orb
  if (!ublox_ubx_msgs__msg__OtherOrbInfo__init(&msg->other_orb)) {
    ublox_ubx_msgs__msg__OrbSVInfo__fini(msg);
    return false;
  }
  return true;
}

void
ublox_ubx_msgs__msg__OrbSVInfo__fini(ublox_ubx_msgs__msg__OrbSVInfo * msg)
{
  if (!msg) {
    return;
  }
  // gnss_id
  // sv_id
  // sv_flag
  ublox_ubx_msgs__msg__OrbSVFlag__fini(&msg->sv_flag);
  // eph
  ublox_ubx_msgs__msg__OrbEphInfo__fini(&msg->eph);
  // alm
  ublox_ubx_msgs__msg__OrbAlmInfo__fini(&msg->alm);
  // other_orb
  ublox_ubx_msgs__msg__OtherOrbInfo__fini(&msg->other_orb);
}

bool
ublox_ubx_msgs__msg__OrbSVInfo__are_equal(const ublox_ubx_msgs__msg__OrbSVInfo * lhs, const ublox_ubx_msgs__msg__OrbSVInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // gnss_id
  if (lhs->gnss_id != rhs->gnss_id) {
    return false;
  }
  // sv_id
  if (lhs->sv_id != rhs->sv_id) {
    return false;
  }
  // sv_flag
  if (!ublox_ubx_msgs__msg__OrbSVFlag__are_equal(
      &(lhs->sv_flag), &(rhs->sv_flag)))
  {
    return false;
  }
  // eph
  if (!ublox_ubx_msgs__msg__OrbEphInfo__are_equal(
      &(lhs->eph), &(rhs->eph)))
  {
    return false;
  }
  // alm
  if (!ublox_ubx_msgs__msg__OrbAlmInfo__are_equal(
      &(lhs->alm), &(rhs->alm)))
  {
    return false;
  }
  // other_orb
  if (!ublox_ubx_msgs__msg__OtherOrbInfo__are_equal(
      &(lhs->other_orb), &(rhs->other_orb)))
  {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__OrbSVInfo__copy(
  const ublox_ubx_msgs__msg__OrbSVInfo * input,
  ublox_ubx_msgs__msg__OrbSVInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // gnss_id
  output->gnss_id = input->gnss_id;
  // sv_id
  output->sv_id = input->sv_id;
  // sv_flag
  if (!ublox_ubx_msgs__msg__OrbSVFlag__copy(
      &(input->sv_flag), &(output->sv_flag)))
  {
    return false;
  }
  // eph
  if (!ublox_ubx_msgs__msg__OrbEphInfo__copy(
      &(input->eph), &(output->eph)))
  {
    return false;
  }
  // alm
  if (!ublox_ubx_msgs__msg__OrbAlmInfo__copy(
      &(input->alm), &(output->alm)))
  {
    return false;
  }
  // other_orb
  if (!ublox_ubx_msgs__msg__OtherOrbInfo__copy(
      &(input->other_orb), &(output->other_orb)))
  {
    return false;
  }
  return true;
}

ublox_ubx_msgs__msg__OrbSVInfo *
ublox_ubx_msgs__msg__OrbSVInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__OrbSVInfo * msg = (ublox_ubx_msgs__msg__OrbSVInfo *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__OrbSVInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__OrbSVInfo));
  bool success = ublox_ubx_msgs__msg__OrbSVInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__OrbSVInfo__destroy(ublox_ubx_msgs__msg__OrbSVInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__OrbSVInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__OrbSVInfo__Sequence__init(ublox_ubx_msgs__msg__OrbSVInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__OrbSVInfo * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__OrbSVInfo *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__OrbSVInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__OrbSVInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__OrbSVInfo__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__OrbSVInfo__Sequence__fini(ublox_ubx_msgs__msg__OrbSVInfo__Sequence * array)
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
      ublox_ubx_msgs__msg__OrbSVInfo__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__OrbSVInfo__Sequence *
ublox_ubx_msgs__msg__OrbSVInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__OrbSVInfo__Sequence * array = (ublox_ubx_msgs__msg__OrbSVInfo__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__OrbSVInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__OrbSVInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__OrbSVInfo__Sequence__destroy(ublox_ubx_msgs__msg__OrbSVInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__OrbSVInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__OrbSVInfo__Sequence__are_equal(const ublox_ubx_msgs__msg__OrbSVInfo__Sequence * lhs, const ublox_ubx_msgs__msg__OrbSVInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__OrbSVInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__OrbSVInfo__Sequence__copy(
  const ublox_ubx_msgs__msg__OrbSVInfo__Sequence * input,
  ublox_ubx_msgs__msg__OrbSVInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__OrbSVInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__OrbSVInfo * data =
      (ublox_ubx_msgs__msg__OrbSVInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__OrbSVInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__OrbSVInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__OrbSVInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
