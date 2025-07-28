// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosECEF.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavHPPosECEF__init(ublox_ubx_msgs__msg__UBXNavHPPosECEF * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavHPPosECEF__fini(msg);
    return false;
  }
  // version
  // itow
  // ecef_x
  // ecef_y
  // ecef_z
  // ecef_x_hp
  // ecef_y_hp
  // ecef_z_hp
  // invalid_ecef_x
  // invalid_ecef_y
  // invalid_ecef_z
  // invalid_ecef_x_hp
  // invalid_ecef_y_hp
  // invalid_ecef_z_hp
  // p_acc
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavHPPosECEF__fini(ublox_ubx_msgs__msg__UBXNavHPPosECEF * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // version
  // itow
  // ecef_x
  // ecef_y
  // ecef_z
  // ecef_x_hp
  // ecef_y_hp
  // ecef_z_hp
  // invalid_ecef_x
  // invalid_ecef_y
  // invalid_ecef_z
  // invalid_ecef_x_hp
  // invalid_ecef_y_hp
  // invalid_ecef_z_hp
  // p_acc
}

bool
ublox_ubx_msgs__msg__UBXNavHPPosECEF__are_equal(const ublox_ubx_msgs__msg__UBXNavHPPosECEF * lhs, const ublox_ubx_msgs__msg__UBXNavHPPosECEF * rhs)
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
  // ecef_x
  if (lhs->ecef_x != rhs->ecef_x) {
    return false;
  }
  // ecef_y
  if (lhs->ecef_y != rhs->ecef_y) {
    return false;
  }
  // ecef_z
  if (lhs->ecef_z != rhs->ecef_z) {
    return false;
  }
  // ecef_x_hp
  if (lhs->ecef_x_hp != rhs->ecef_x_hp) {
    return false;
  }
  // ecef_y_hp
  if (lhs->ecef_y_hp != rhs->ecef_y_hp) {
    return false;
  }
  // ecef_z_hp
  if (lhs->ecef_z_hp != rhs->ecef_z_hp) {
    return false;
  }
  // invalid_ecef_x
  if (lhs->invalid_ecef_x != rhs->invalid_ecef_x) {
    return false;
  }
  // invalid_ecef_y
  if (lhs->invalid_ecef_y != rhs->invalid_ecef_y) {
    return false;
  }
  // invalid_ecef_z
  if (lhs->invalid_ecef_z != rhs->invalid_ecef_z) {
    return false;
  }
  // invalid_ecef_x_hp
  if (lhs->invalid_ecef_x_hp != rhs->invalid_ecef_x_hp) {
    return false;
  }
  // invalid_ecef_y_hp
  if (lhs->invalid_ecef_y_hp != rhs->invalid_ecef_y_hp) {
    return false;
  }
  // invalid_ecef_z_hp
  if (lhs->invalid_ecef_z_hp != rhs->invalid_ecef_z_hp) {
    return false;
  }
  // p_acc
  if (lhs->p_acc != rhs->p_acc) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavHPPosECEF__copy(
  const ublox_ubx_msgs__msg__UBXNavHPPosECEF * input,
  ublox_ubx_msgs__msg__UBXNavHPPosECEF * output)
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
  // ecef_x
  output->ecef_x = input->ecef_x;
  // ecef_y
  output->ecef_y = input->ecef_y;
  // ecef_z
  output->ecef_z = input->ecef_z;
  // ecef_x_hp
  output->ecef_x_hp = input->ecef_x_hp;
  // ecef_y_hp
  output->ecef_y_hp = input->ecef_y_hp;
  // ecef_z_hp
  output->ecef_z_hp = input->ecef_z_hp;
  // invalid_ecef_x
  output->invalid_ecef_x = input->invalid_ecef_x;
  // invalid_ecef_y
  output->invalid_ecef_y = input->invalid_ecef_y;
  // invalid_ecef_z
  output->invalid_ecef_z = input->invalid_ecef_z;
  // invalid_ecef_x_hp
  output->invalid_ecef_x_hp = input->invalid_ecef_x_hp;
  // invalid_ecef_y_hp
  output->invalid_ecef_y_hp = input->invalid_ecef_y_hp;
  // invalid_ecef_z_hp
  output->invalid_ecef_z_hp = input->invalid_ecef_z_hp;
  // p_acc
  output->p_acc = input->p_acc;
  return true;
}

ublox_ubx_msgs__msg__UBXNavHPPosECEF *
ublox_ubx_msgs__msg__UBXNavHPPosECEF__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavHPPosECEF * msg = (ublox_ubx_msgs__msg__UBXNavHPPosECEF *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavHPPosECEF), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavHPPosECEF));
  bool success = ublox_ubx_msgs__msg__UBXNavHPPosECEF__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavHPPosECEF__destroy(ublox_ubx_msgs__msg__UBXNavHPPosECEF * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavHPPosECEF__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence__init(ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavHPPosECEF * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavHPPosECEF *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavHPPosECEF), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavHPPosECEF__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavHPPosECEF__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence__fini(ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavHPPosECEF__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence *
ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence * array = (ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavHPPosECEF__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavHPPosECEF);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavHPPosECEF * data =
      (ublox_ubx_msgs__msg__UBXNavHPPosECEF *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavHPPosECEF__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavHPPosECEF__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavHPPosECEF__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
