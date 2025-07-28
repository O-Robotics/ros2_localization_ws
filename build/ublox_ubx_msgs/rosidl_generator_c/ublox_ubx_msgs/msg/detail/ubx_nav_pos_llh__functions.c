// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavPosLLH.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_pos_llh__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavPosLLH__init(ublox_ubx_msgs__msg__UBXNavPosLLH * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavPosLLH__fini(msg);
    return false;
  }
  // itow
  // lon
  // lat
  // height
  // hmsl
  // h_acc
  // v_acc
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavPosLLH__fini(ublox_ubx_msgs__msg__UBXNavPosLLH * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // itow
  // lon
  // lat
  // height
  // hmsl
  // h_acc
  // v_acc
}

bool
ublox_ubx_msgs__msg__UBXNavPosLLH__are_equal(const ublox_ubx_msgs__msg__UBXNavPosLLH * lhs, const ublox_ubx_msgs__msg__UBXNavPosLLH * rhs)
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
  // lon
  if (lhs->lon != rhs->lon) {
    return false;
  }
  // lat
  if (lhs->lat != rhs->lat) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // hmsl
  if (lhs->hmsl != rhs->hmsl) {
    return false;
  }
  // h_acc
  if (lhs->h_acc != rhs->h_acc) {
    return false;
  }
  // v_acc
  if (lhs->v_acc != rhs->v_acc) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavPosLLH__copy(
  const ublox_ubx_msgs__msg__UBXNavPosLLH * input,
  ublox_ubx_msgs__msg__UBXNavPosLLH * output)
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
  // lon
  output->lon = input->lon;
  // lat
  output->lat = input->lat;
  // height
  output->height = input->height;
  // hmsl
  output->hmsl = input->hmsl;
  // h_acc
  output->h_acc = input->h_acc;
  // v_acc
  output->v_acc = input->v_acc;
  return true;
}

ublox_ubx_msgs__msg__UBXNavPosLLH *
ublox_ubx_msgs__msg__UBXNavPosLLH__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavPosLLH * msg = (ublox_ubx_msgs__msg__UBXNavPosLLH *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavPosLLH), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavPosLLH));
  bool success = ublox_ubx_msgs__msg__UBXNavPosLLH__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavPosLLH__destroy(ublox_ubx_msgs__msg__UBXNavPosLLH * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavPosLLH__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence__init(ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavPosLLH * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavPosLLH *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavPosLLH), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavPosLLH__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavPosLLH__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence__fini(ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavPosLLH__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence *
ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence * array = (ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavPosLLH__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavPosLLH);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavPosLLH * data =
      (ublox_ubx_msgs__msg__UBXNavPosLLH *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavPosLLH__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavPosLLH__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavPosLLH__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
