// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/SBASSvData.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ublox_ubx_msgs__msg__SBASSvData__init(ublox_ubx_msgs__msg__SBASSvData * msg)
{
  if (!msg) {
    return false;
  }
  // svid
  // reserved_1
  // udre
  // sv_sys
  // sv_service
  // reserved_2
  // prc
  // reserved_3
  // ic
  return true;
}

void
ublox_ubx_msgs__msg__SBASSvData__fini(ublox_ubx_msgs__msg__SBASSvData * msg)
{
  if (!msg) {
    return;
  }
  // svid
  // reserved_1
  // udre
  // sv_sys
  // sv_service
  // reserved_2
  // prc
  // reserved_3
  // ic
}

bool
ublox_ubx_msgs__msg__SBASSvData__are_equal(const ublox_ubx_msgs__msg__SBASSvData * lhs, const ublox_ubx_msgs__msg__SBASSvData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // svid
  if (lhs->svid != rhs->svid) {
    return false;
  }
  // reserved_1
  if (lhs->reserved_1 != rhs->reserved_1) {
    return false;
  }
  // udre
  if (lhs->udre != rhs->udre) {
    return false;
  }
  // sv_sys
  if (lhs->sv_sys != rhs->sv_sys) {
    return false;
  }
  // sv_service
  if (lhs->sv_service != rhs->sv_service) {
    return false;
  }
  // reserved_2
  if (lhs->reserved_2 != rhs->reserved_2) {
    return false;
  }
  // prc
  if (lhs->prc != rhs->prc) {
    return false;
  }
  // reserved_3
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->reserved_3[i] != rhs->reserved_3[i]) {
      return false;
    }
  }
  // ic
  if (lhs->ic != rhs->ic) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SBASSvData__copy(
  const ublox_ubx_msgs__msg__SBASSvData * input,
  ublox_ubx_msgs__msg__SBASSvData * output)
{
  if (!input || !output) {
    return false;
  }
  // svid
  output->svid = input->svid;
  // reserved_1
  output->reserved_1 = input->reserved_1;
  // udre
  output->udre = input->udre;
  // sv_sys
  output->sv_sys = input->sv_sys;
  // sv_service
  output->sv_service = input->sv_service;
  // reserved_2
  output->reserved_2 = input->reserved_2;
  // prc
  output->prc = input->prc;
  // reserved_3
  for (size_t i = 0; i < 2; ++i) {
    output->reserved_3[i] = input->reserved_3[i];
  }
  // ic
  output->ic = input->ic;
  return true;
}

ublox_ubx_msgs__msg__SBASSvData *
ublox_ubx_msgs__msg__SBASSvData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SBASSvData * msg = (ublox_ubx_msgs__msg__SBASSvData *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SBASSvData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__SBASSvData));
  bool success = ublox_ubx_msgs__msg__SBASSvData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__SBASSvData__destroy(ublox_ubx_msgs__msg__SBASSvData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__SBASSvData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__SBASSvData__Sequence__init(ublox_ubx_msgs__msg__SBASSvData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SBASSvData * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__SBASSvData *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__SBASSvData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__SBASSvData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__SBASSvData__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__SBASSvData__Sequence__fini(ublox_ubx_msgs__msg__SBASSvData__Sequence * array)
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
      ublox_ubx_msgs__msg__SBASSvData__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__SBASSvData__Sequence *
ublox_ubx_msgs__msg__SBASSvData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SBASSvData__Sequence * array = (ublox_ubx_msgs__msg__SBASSvData__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SBASSvData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__SBASSvData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__SBASSvData__Sequence__destroy(ublox_ubx_msgs__msg__SBASSvData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__SBASSvData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__SBASSvData__Sequence__are_equal(const ublox_ubx_msgs__msg__SBASSvData__Sequence * lhs, const ublox_ubx_msgs__msg__SBASSvData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__SBASSvData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SBASSvData__Sequence__copy(
  const ublox_ubx_msgs__msg__SBASSvData__Sequence * input,
  ublox_ubx_msgs__msg__SBASSvData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__SBASSvData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__SBASSvData * data =
      (ublox_ubx_msgs__msg__SBASSvData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__SBASSvData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__SBASSvData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__SBASSvData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
