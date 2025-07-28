// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavCov.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_cov__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavCov__init(ublox_ubx_msgs__msg__UBXNavCov * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavCov__fini(msg);
    return false;
  }
  // itow
  // version
  // pos_cor_valid
  // vel_cor_valid
  // pos_cov_nn
  // pos_cov_ne
  // pos_cov_nd
  // pos_cov_ee
  // pos_cov_ed
  // pos_cov_dd
  // vel_cov_nn
  // vel_cov_ne
  // vel_cov_nd
  // vel_cov_ee
  // vel_cov_ed
  // vel_cov_dd
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavCov__fini(ublox_ubx_msgs__msg__UBXNavCov * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // itow
  // version
  // pos_cor_valid
  // vel_cor_valid
  // pos_cov_nn
  // pos_cov_ne
  // pos_cov_nd
  // pos_cov_ee
  // pos_cov_ed
  // pos_cov_dd
  // vel_cov_nn
  // vel_cov_ne
  // vel_cov_nd
  // vel_cov_ee
  // vel_cov_ed
  // vel_cov_dd
}

bool
ublox_ubx_msgs__msg__UBXNavCov__are_equal(const ublox_ubx_msgs__msg__UBXNavCov * lhs, const ublox_ubx_msgs__msg__UBXNavCov * rhs)
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
  // pos_cor_valid
  if (lhs->pos_cor_valid != rhs->pos_cor_valid) {
    return false;
  }
  // vel_cor_valid
  if (lhs->vel_cor_valid != rhs->vel_cor_valid) {
    return false;
  }
  // pos_cov_nn
  if (lhs->pos_cov_nn != rhs->pos_cov_nn) {
    return false;
  }
  // pos_cov_ne
  if (lhs->pos_cov_ne != rhs->pos_cov_ne) {
    return false;
  }
  // pos_cov_nd
  if (lhs->pos_cov_nd != rhs->pos_cov_nd) {
    return false;
  }
  // pos_cov_ee
  if (lhs->pos_cov_ee != rhs->pos_cov_ee) {
    return false;
  }
  // pos_cov_ed
  if (lhs->pos_cov_ed != rhs->pos_cov_ed) {
    return false;
  }
  // pos_cov_dd
  if (lhs->pos_cov_dd != rhs->pos_cov_dd) {
    return false;
  }
  // vel_cov_nn
  if (lhs->vel_cov_nn != rhs->vel_cov_nn) {
    return false;
  }
  // vel_cov_ne
  if (lhs->vel_cov_ne != rhs->vel_cov_ne) {
    return false;
  }
  // vel_cov_nd
  if (lhs->vel_cov_nd != rhs->vel_cov_nd) {
    return false;
  }
  // vel_cov_ee
  if (lhs->vel_cov_ee != rhs->vel_cov_ee) {
    return false;
  }
  // vel_cov_ed
  if (lhs->vel_cov_ed != rhs->vel_cov_ed) {
    return false;
  }
  // vel_cov_dd
  if (lhs->vel_cov_dd != rhs->vel_cov_dd) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavCov__copy(
  const ublox_ubx_msgs__msg__UBXNavCov * input,
  ublox_ubx_msgs__msg__UBXNavCov * output)
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
  // pos_cor_valid
  output->pos_cor_valid = input->pos_cor_valid;
  // vel_cor_valid
  output->vel_cor_valid = input->vel_cor_valid;
  // pos_cov_nn
  output->pos_cov_nn = input->pos_cov_nn;
  // pos_cov_ne
  output->pos_cov_ne = input->pos_cov_ne;
  // pos_cov_nd
  output->pos_cov_nd = input->pos_cov_nd;
  // pos_cov_ee
  output->pos_cov_ee = input->pos_cov_ee;
  // pos_cov_ed
  output->pos_cov_ed = input->pos_cov_ed;
  // pos_cov_dd
  output->pos_cov_dd = input->pos_cov_dd;
  // vel_cov_nn
  output->vel_cov_nn = input->vel_cov_nn;
  // vel_cov_ne
  output->vel_cov_ne = input->vel_cov_ne;
  // vel_cov_nd
  output->vel_cov_nd = input->vel_cov_nd;
  // vel_cov_ee
  output->vel_cov_ee = input->vel_cov_ee;
  // vel_cov_ed
  output->vel_cov_ed = input->vel_cov_ed;
  // vel_cov_dd
  output->vel_cov_dd = input->vel_cov_dd;
  return true;
}

ublox_ubx_msgs__msg__UBXNavCov *
ublox_ubx_msgs__msg__UBXNavCov__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavCov * msg = (ublox_ubx_msgs__msg__UBXNavCov *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavCov), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavCov));
  bool success = ublox_ubx_msgs__msg__UBXNavCov__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavCov__destroy(ublox_ubx_msgs__msg__UBXNavCov * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavCov__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavCov__Sequence__init(ublox_ubx_msgs__msg__UBXNavCov__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavCov * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavCov *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavCov), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavCov__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavCov__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavCov__Sequence__fini(ublox_ubx_msgs__msg__UBXNavCov__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavCov__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavCov__Sequence *
ublox_ubx_msgs__msg__UBXNavCov__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavCov__Sequence * array = (ublox_ubx_msgs__msg__UBXNavCov__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavCov__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavCov__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavCov__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavCov__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavCov__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavCov__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavCov__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavCov__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavCov__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavCov__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavCov__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavCov__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavCov);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavCov * data =
      (ublox_ubx_msgs__msg__UBXNavCov *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavCov__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavCov__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavCov__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
