// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavSBAS.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_sbas__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `service`
#include "ublox_ubx_msgs/msg/detail/sbas_service__functions.h"
// Member `status_flags`
#include "ublox_ubx_msgs/msg/detail/sbas_status_flags__functions.h"
// Member `sv_data`
#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavSBAS__init(ublox_ubx_msgs__msg__UBXNavSBAS * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavSBAS__fini(msg);
    return false;
  }
  // itow
  // geo
  // mode
  // sys
  // service
  if (!ublox_ubx_msgs__msg__SBASService__init(&msg->service)) {
    ublox_ubx_msgs__msg__UBXNavSBAS__fini(msg);
    return false;
  }
  // cnt
  // status_flags
  if (!ublox_ubx_msgs__msg__SBASStatusFlags__init(&msg->status_flags)) {
    ublox_ubx_msgs__msg__UBXNavSBAS__fini(msg);
    return false;
  }
  // reserved_0
  // sv_data
  if (!ublox_ubx_msgs__msg__SBASSvData__Sequence__init(&msg->sv_data, 0)) {
    ublox_ubx_msgs__msg__UBXNavSBAS__fini(msg);
    return false;
  }
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavSBAS__fini(ublox_ubx_msgs__msg__UBXNavSBAS * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // itow
  // geo
  // mode
  // sys
  // service
  ublox_ubx_msgs__msg__SBASService__fini(&msg->service);
  // cnt
  // status_flags
  ublox_ubx_msgs__msg__SBASStatusFlags__fini(&msg->status_flags);
  // reserved_0
  // sv_data
  ublox_ubx_msgs__msg__SBASSvData__Sequence__fini(&msg->sv_data);
}

bool
ublox_ubx_msgs__msg__UBXNavSBAS__are_equal(const ublox_ubx_msgs__msg__UBXNavSBAS * lhs, const ublox_ubx_msgs__msg__UBXNavSBAS * rhs)
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
  // geo
  if (lhs->geo != rhs->geo) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // sys
  if (lhs->sys != rhs->sys) {
    return false;
  }
  // service
  if (!ublox_ubx_msgs__msg__SBASService__are_equal(
      &(lhs->service), &(rhs->service)))
  {
    return false;
  }
  // cnt
  if (lhs->cnt != rhs->cnt) {
    return false;
  }
  // status_flags
  if (!ublox_ubx_msgs__msg__SBASStatusFlags__are_equal(
      &(lhs->status_flags), &(rhs->status_flags)))
  {
    return false;
  }
  // reserved_0
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->reserved_0[i] != rhs->reserved_0[i]) {
      return false;
    }
  }
  // sv_data
  if (!ublox_ubx_msgs__msg__SBASSvData__Sequence__are_equal(
      &(lhs->sv_data), &(rhs->sv_data)))
  {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavSBAS__copy(
  const ublox_ubx_msgs__msg__UBXNavSBAS * input,
  ublox_ubx_msgs__msg__UBXNavSBAS * output)
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
  // geo
  output->geo = input->geo;
  // mode
  output->mode = input->mode;
  // sys
  output->sys = input->sys;
  // service
  if (!ublox_ubx_msgs__msg__SBASService__copy(
      &(input->service), &(output->service)))
  {
    return false;
  }
  // cnt
  output->cnt = input->cnt;
  // status_flags
  if (!ublox_ubx_msgs__msg__SBASStatusFlags__copy(
      &(input->status_flags), &(output->status_flags)))
  {
    return false;
  }
  // reserved_0
  for (size_t i = 0; i < 2; ++i) {
    output->reserved_0[i] = input->reserved_0[i];
  }
  // sv_data
  if (!ublox_ubx_msgs__msg__SBASSvData__Sequence__copy(
      &(input->sv_data), &(output->sv_data)))
  {
    return false;
  }
  return true;
}

ublox_ubx_msgs__msg__UBXNavSBAS *
ublox_ubx_msgs__msg__UBXNavSBAS__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavSBAS * msg = (ublox_ubx_msgs__msg__UBXNavSBAS *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavSBAS), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavSBAS));
  bool success = ublox_ubx_msgs__msg__UBXNavSBAS__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavSBAS__destroy(ublox_ubx_msgs__msg__UBXNavSBAS * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavSBAS__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavSBAS__Sequence__init(ublox_ubx_msgs__msg__UBXNavSBAS__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavSBAS * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavSBAS *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavSBAS), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavSBAS__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavSBAS__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavSBAS__Sequence__fini(ublox_ubx_msgs__msg__UBXNavSBAS__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavSBAS__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavSBAS__Sequence *
ublox_ubx_msgs__msg__UBXNavSBAS__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavSBAS__Sequence * array = (ublox_ubx_msgs__msg__UBXNavSBAS__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavSBAS__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavSBAS__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavSBAS__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavSBAS__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavSBAS__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavSBAS__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavSBAS__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavSBAS__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavSBAS__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavSBAS__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavSBAS__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavSBAS__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavSBAS);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavSBAS * data =
      (ublox_ubx_msgs__msg__UBXNavSBAS *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavSBAS__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavSBAS__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavSBAS__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
