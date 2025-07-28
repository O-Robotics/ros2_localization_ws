// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosLLH.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_llh__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavHPPosLLH__init(ublox_ubx_msgs__msg__UBXNavHPPosLLH * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavHPPosLLH__fini(msg);
    return false;
  }
  // version
  // invalid_lon
  // invalid_lat
  // invalid_height
  // invalid_hmsl
  // invalid_lon_hp
  // invalid_lat_hp
  // invalid_height_hp
  // invalid_hmsl_hp
  // itow
  // lon
  // lat
  // height
  // hmsl
  // lon_hp
  // lat_hp
  // height_hp
  // hmsl_hp
  // h_acc
  // v_acc
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavHPPosLLH__fini(ublox_ubx_msgs__msg__UBXNavHPPosLLH * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // version
  // invalid_lon
  // invalid_lat
  // invalid_height
  // invalid_hmsl
  // invalid_lon_hp
  // invalid_lat_hp
  // invalid_height_hp
  // invalid_hmsl_hp
  // itow
  // lon
  // lat
  // height
  // hmsl
  // lon_hp
  // lat_hp
  // height_hp
  // hmsl_hp
  // h_acc
  // v_acc
}

bool
ublox_ubx_msgs__msg__UBXNavHPPosLLH__are_equal(const ublox_ubx_msgs__msg__UBXNavHPPosLLH * lhs, const ublox_ubx_msgs__msg__UBXNavHPPosLLH * rhs)
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
  // invalid_lon
  if (lhs->invalid_lon != rhs->invalid_lon) {
    return false;
  }
  // invalid_lat
  if (lhs->invalid_lat != rhs->invalid_lat) {
    return false;
  }
  // invalid_height
  if (lhs->invalid_height != rhs->invalid_height) {
    return false;
  }
  // invalid_hmsl
  if (lhs->invalid_hmsl != rhs->invalid_hmsl) {
    return false;
  }
  // invalid_lon_hp
  if (lhs->invalid_lon_hp != rhs->invalid_lon_hp) {
    return false;
  }
  // invalid_lat_hp
  if (lhs->invalid_lat_hp != rhs->invalid_lat_hp) {
    return false;
  }
  // invalid_height_hp
  if (lhs->invalid_height_hp != rhs->invalid_height_hp) {
    return false;
  }
  // invalid_hmsl_hp
  if (lhs->invalid_hmsl_hp != rhs->invalid_hmsl_hp) {
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
  // lon_hp
  if (lhs->lon_hp != rhs->lon_hp) {
    return false;
  }
  // lat_hp
  if (lhs->lat_hp != rhs->lat_hp) {
    return false;
  }
  // height_hp
  if (lhs->height_hp != rhs->height_hp) {
    return false;
  }
  // hmsl_hp
  if (lhs->hmsl_hp != rhs->hmsl_hp) {
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
ublox_ubx_msgs__msg__UBXNavHPPosLLH__copy(
  const ublox_ubx_msgs__msg__UBXNavHPPosLLH * input,
  ublox_ubx_msgs__msg__UBXNavHPPosLLH * output)
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
  // invalid_lon
  output->invalid_lon = input->invalid_lon;
  // invalid_lat
  output->invalid_lat = input->invalid_lat;
  // invalid_height
  output->invalid_height = input->invalid_height;
  // invalid_hmsl
  output->invalid_hmsl = input->invalid_hmsl;
  // invalid_lon_hp
  output->invalid_lon_hp = input->invalid_lon_hp;
  // invalid_lat_hp
  output->invalid_lat_hp = input->invalid_lat_hp;
  // invalid_height_hp
  output->invalid_height_hp = input->invalid_height_hp;
  // invalid_hmsl_hp
  output->invalid_hmsl_hp = input->invalid_hmsl_hp;
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
  // lon_hp
  output->lon_hp = input->lon_hp;
  // lat_hp
  output->lat_hp = input->lat_hp;
  // height_hp
  output->height_hp = input->height_hp;
  // hmsl_hp
  output->hmsl_hp = input->hmsl_hp;
  // h_acc
  output->h_acc = input->h_acc;
  // v_acc
  output->v_acc = input->v_acc;
  return true;
}

ublox_ubx_msgs__msg__UBXNavHPPosLLH *
ublox_ubx_msgs__msg__UBXNavHPPosLLH__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavHPPosLLH * msg = (ublox_ubx_msgs__msg__UBXNavHPPosLLH *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavHPPosLLH), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavHPPosLLH));
  bool success = ublox_ubx_msgs__msg__UBXNavHPPosLLH__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavHPPosLLH__destroy(ublox_ubx_msgs__msg__UBXNavHPPosLLH * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavHPPosLLH__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence__init(ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavHPPosLLH * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavHPPosLLH *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavHPPosLLH), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavHPPosLLH__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavHPPosLLH__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence__fini(ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavHPPosLLH__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence *
ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence * array = (ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavHPPosLLH__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavHPPosLLH);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavHPPosLLH * data =
      (ublox_ubx_msgs__msg__UBXNavHPPosLLH *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavHPPosLLH__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavHPPosLLH__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavHPPosLLH__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
