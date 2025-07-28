// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/SatInfo.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sat_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `flags`
#include "ublox_ubx_msgs/msg/detail/sat_flags__functions.h"

bool
ublox_ubx_msgs__msg__SatInfo__init(ublox_ubx_msgs__msg__SatInfo * msg)
{
  if (!msg) {
    return false;
  }
  // gnss_id
  // sv_id
  // cno
  // elev
  // azim
  // pr_res
  // flags
  if (!ublox_ubx_msgs__msg__SatFlags__init(&msg->flags)) {
    ublox_ubx_msgs__msg__SatInfo__fini(msg);
    return false;
  }
  return true;
}

void
ublox_ubx_msgs__msg__SatInfo__fini(ublox_ubx_msgs__msg__SatInfo * msg)
{
  if (!msg) {
    return;
  }
  // gnss_id
  // sv_id
  // cno
  // elev
  // azim
  // pr_res
  // flags
  ublox_ubx_msgs__msg__SatFlags__fini(&msg->flags);
}

bool
ublox_ubx_msgs__msg__SatInfo__are_equal(const ublox_ubx_msgs__msg__SatInfo * lhs, const ublox_ubx_msgs__msg__SatInfo * rhs)
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
  // cno
  if (lhs->cno != rhs->cno) {
    return false;
  }
  // elev
  if (lhs->elev != rhs->elev) {
    return false;
  }
  // azim
  if (lhs->azim != rhs->azim) {
    return false;
  }
  // pr_res
  if (lhs->pr_res != rhs->pr_res) {
    return false;
  }
  // flags
  if (!ublox_ubx_msgs__msg__SatFlags__are_equal(
      &(lhs->flags), &(rhs->flags)))
  {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SatInfo__copy(
  const ublox_ubx_msgs__msg__SatInfo * input,
  ublox_ubx_msgs__msg__SatInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // gnss_id
  output->gnss_id = input->gnss_id;
  // sv_id
  output->sv_id = input->sv_id;
  // cno
  output->cno = input->cno;
  // elev
  output->elev = input->elev;
  // azim
  output->azim = input->azim;
  // pr_res
  output->pr_res = input->pr_res;
  // flags
  if (!ublox_ubx_msgs__msg__SatFlags__copy(
      &(input->flags), &(output->flags)))
  {
    return false;
  }
  return true;
}

ublox_ubx_msgs__msg__SatInfo *
ublox_ubx_msgs__msg__SatInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SatInfo * msg = (ublox_ubx_msgs__msg__SatInfo *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SatInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__SatInfo));
  bool success = ublox_ubx_msgs__msg__SatInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__SatInfo__destroy(ublox_ubx_msgs__msg__SatInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__SatInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__SatInfo__Sequence__init(ublox_ubx_msgs__msg__SatInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SatInfo * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__SatInfo *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__SatInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__SatInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__SatInfo__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__SatInfo__Sequence__fini(ublox_ubx_msgs__msg__SatInfo__Sequence * array)
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
      ublox_ubx_msgs__msg__SatInfo__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__SatInfo__Sequence *
ublox_ubx_msgs__msg__SatInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SatInfo__Sequence * array = (ublox_ubx_msgs__msg__SatInfo__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SatInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__SatInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__SatInfo__Sequence__destroy(ublox_ubx_msgs__msg__SatInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__SatInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__SatInfo__Sequence__are_equal(const ublox_ubx_msgs__msg__SatInfo__Sequence * lhs, const ublox_ubx_msgs__msg__SatInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__SatInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SatInfo__Sequence__copy(
  const ublox_ubx_msgs__msg__SatInfo__Sequence * input,
  ublox_ubx_msgs__msg__SatInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__SatInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__SatInfo * data =
      (ublox_ubx_msgs__msg__SatInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__SatInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__SatInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__SatInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
