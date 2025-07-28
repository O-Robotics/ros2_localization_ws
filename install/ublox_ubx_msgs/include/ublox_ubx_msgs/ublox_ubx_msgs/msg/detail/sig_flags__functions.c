// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/SigFlags.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sig_flags__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ublox_ubx_msgs__msg__SigFlags__init(ublox_ubx_msgs__msg__SigFlags * msg)
{
  if (!msg) {
    return false;
  }
  // health
  // pr_smoothed
  // pr_used
  // cr_used
  // do_used
  // pr_corr_used
  // cr_corr_used
  // do_corr_used
  return true;
}

void
ublox_ubx_msgs__msg__SigFlags__fini(ublox_ubx_msgs__msg__SigFlags * msg)
{
  if (!msg) {
    return;
  }
  // health
  // pr_smoothed
  // pr_used
  // cr_used
  // do_used
  // pr_corr_used
  // cr_corr_used
  // do_corr_used
}

bool
ublox_ubx_msgs__msg__SigFlags__are_equal(const ublox_ubx_msgs__msg__SigFlags * lhs, const ublox_ubx_msgs__msg__SigFlags * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // health
  if (lhs->health != rhs->health) {
    return false;
  }
  // pr_smoothed
  if (lhs->pr_smoothed != rhs->pr_smoothed) {
    return false;
  }
  // pr_used
  if (lhs->pr_used != rhs->pr_used) {
    return false;
  }
  // cr_used
  if (lhs->cr_used != rhs->cr_used) {
    return false;
  }
  // do_used
  if (lhs->do_used != rhs->do_used) {
    return false;
  }
  // pr_corr_used
  if (lhs->pr_corr_used != rhs->pr_corr_used) {
    return false;
  }
  // cr_corr_used
  if (lhs->cr_corr_used != rhs->cr_corr_used) {
    return false;
  }
  // do_corr_used
  if (lhs->do_corr_used != rhs->do_corr_used) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SigFlags__copy(
  const ublox_ubx_msgs__msg__SigFlags * input,
  ublox_ubx_msgs__msg__SigFlags * output)
{
  if (!input || !output) {
    return false;
  }
  // health
  output->health = input->health;
  // pr_smoothed
  output->pr_smoothed = input->pr_smoothed;
  // pr_used
  output->pr_used = input->pr_used;
  // cr_used
  output->cr_used = input->cr_used;
  // do_used
  output->do_used = input->do_used;
  // pr_corr_used
  output->pr_corr_used = input->pr_corr_used;
  // cr_corr_used
  output->cr_corr_used = input->cr_corr_used;
  // do_corr_used
  output->do_corr_used = input->do_corr_used;
  return true;
}

ublox_ubx_msgs__msg__SigFlags *
ublox_ubx_msgs__msg__SigFlags__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SigFlags * msg = (ublox_ubx_msgs__msg__SigFlags *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SigFlags), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__SigFlags));
  bool success = ublox_ubx_msgs__msg__SigFlags__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__SigFlags__destroy(ublox_ubx_msgs__msg__SigFlags * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__SigFlags__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__SigFlags__Sequence__init(ublox_ubx_msgs__msg__SigFlags__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SigFlags * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__SigFlags *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__SigFlags), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__SigFlags__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__SigFlags__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__SigFlags__Sequence__fini(ublox_ubx_msgs__msg__SigFlags__Sequence * array)
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
      ublox_ubx_msgs__msg__SigFlags__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__SigFlags__Sequence *
ublox_ubx_msgs__msg__SigFlags__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SigFlags__Sequence * array = (ublox_ubx_msgs__msg__SigFlags__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SigFlags__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__SigFlags__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__SigFlags__Sequence__destroy(ublox_ubx_msgs__msg__SigFlags__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__SigFlags__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__SigFlags__Sequence__are_equal(const ublox_ubx_msgs__msg__SigFlags__Sequence * lhs, const ublox_ubx_msgs__msg__SigFlags__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__SigFlags__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SigFlags__Sequence__copy(
  const ublox_ubx_msgs__msg__SigFlags__Sequence * input,
  ublox_ubx_msgs__msg__SigFlags__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__SigFlags);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__SigFlags * data =
      (ublox_ubx_msgs__msg__SigFlags *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__SigFlags__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__SigFlags__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__SigFlags__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
