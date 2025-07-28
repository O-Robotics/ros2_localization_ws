// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXRxmRawx.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `rec_stat`
#include "ublox_ubx_msgs/msg/detail/rec_stat__functions.h"
// Member `rawx_data`
#include "ublox_ubx_msgs/msg/detail/rawx_data__functions.h"

bool
ublox_ubx_msgs__msg__UBXRxmRawx__init(ublox_ubx_msgs__msg__UBXRxmRawx * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXRxmRawx__fini(msg);
    return false;
  }
  // rcv_tow
  // week
  // leap_s
  // num_meas
  // rec_stat
  if (!ublox_ubx_msgs__msg__RecStat__init(&msg->rec_stat)) {
    ublox_ubx_msgs__msg__UBXRxmRawx__fini(msg);
    return false;
  }
  // version
  // rawx_data
  if (!ublox_ubx_msgs__msg__RawxData__Sequence__init(&msg->rawx_data, 0)) {
    ublox_ubx_msgs__msg__UBXRxmRawx__fini(msg);
    return false;
  }
  return true;
}

void
ublox_ubx_msgs__msg__UBXRxmRawx__fini(ublox_ubx_msgs__msg__UBXRxmRawx * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // rcv_tow
  // week
  // leap_s
  // num_meas
  // rec_stat
  ublox_ubx_msgs__msg__RecStat__fini(&msg->rec_stat);
  // version
  // rawx_data
  ublox_ubx_msgs__msg__RawxData__Sequence__fini(&msg->rawx_data);
}

bool
ublox_ubx_msgs__msg__UBXRxmRawx__are_equal(const ublox_ubx_msgs__msg__UBXRxmRawx * lhs, const ublox_ubx_msgs__msg__UBXRxmRawx * rhs)
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
  // rcv_tow
  if (lhs->rcv_tow != rhs->rcv_tow) {
    return false;
  }
  // week
  if (lhs->week != rhs->week) {
    return false;
  }
  // leap_s
  if (lhs->leap_s != rhs->leap_s) {
    return false;
  }
  // num_meas
  if (lhs->num_meas != rhs->num_meas) {
    return false;
  }
  // rec_stat
  if (!ublox_ubx_msgs__msg__RecStat__are_equal(
      &(lhs->rec_stat), &(rhs->rec_stat)))
  {
    return false;
  }
  // version
  if (lhs->version != rhs->version) {
    return false;
  }
  // rawx_data
  if (!ublox_ubx_msgs__msg__RawxData__Sequence__are_equal(
      &(lhs->rawx_data), &(rhs->rawx_data)))
  {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXRxmRawx__copy(
  const ublox_ubx_msgs__msg__UBXRxmRawx * input,
  ublox_ubx_msgs__msg__UBXRxmRawx * output)
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
  // rcv_tow
  output->rcv_tow = input->rcv_tow;
  // week
  output->week = input->week;
  // leap_s
  output->leap_s = input->leap_s;
  // num_meas
  output->num_meas = input->num_meas;
  // rec_stat
  if (!ublox_ubx_msgs__msg__RecStat__copy(
      &(input->rec_stat), &(output->rec_stat)))
  {
    return false;
  }
  // version
  output->version = input->version;
  // rawx_data
  if (!ublox_ubx_msgs__msg__RawxData__Sequence__copy(
      &(input->rawx_data), &(output->rawx_data)))
  {
    return false;
  }
  return true;
}

ublox_ubx_msgs__msg__UBXRxmRawx *
ublox_ubx_msgs__msg__UBXRxmRawx__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXRxmRawx * msg = (ublox_ubx_msgs__msg__UBXRxmRawx *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXRxmRawx), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXRxmRawx));
  bool success = ublox_ubx_msgs__msg__UBXRxmRawx__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXRxmRawx__destroy(ublox_ubx_msgs__msg__UBXRxmRawx * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXRxmRawx__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXRxmRawx__Sequence__init(ublox_ubx_msgs__msg__UBXRxmRawx__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXRxmRawx * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXRxmRawx *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXRxmRawx), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXRxmRawx__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXRxmRawx__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXRxmRawx__Sequence__fini(ublox_ubx_msgs__msg__UBXRxmRawx__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXRxmRawx__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXRxmRawx__Sequence *
ublox_ubx_msgs__msg__UBXRxmRawx__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXRxmRawx__Sequence * array = (ublox_ubx_msgs__msg__UBXRxmRawx__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXRxmRawx__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXRxmRawx__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXRxmRawx__Sequence__destroy(ublox_ubx_msgs__msg__UBXRxmRawx__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXRxmRawx__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXRxmRawx__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXRxmRawx__Sequence * lhs, const ublox_ubx_msgs__msg__UBXRxmRawx__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXRxmRawx__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXRxmRawx__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXRxmRawx__Sequence * input,
  ublox_ubx_msgs__msg__UBXRxmRawx__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXRxmRawx);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXRxmRawx * data =
      (ublox_ubx_msgs__msg__UBXRxmRawx *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXRxmRawx__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXRxmRawx__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXRxmRawx__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
