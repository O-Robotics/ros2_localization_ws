// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXRxmMeasx.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_measx__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sv_data`
#include "ublox_ubx_msgs/msg/detail/measx_data__functions.h"

bool
ublox_ubx_msgs__msg__UBXRxmMeasx__init(ublox_ubx_msgs__msg__UBXRxmMeasx * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXRxmMeasx__fini(msg);
    return false;
  }
  // version
  // gps_tow
  // glo_tow
  // bds_tow
  // qzss_tow
  // gps_tow_acc
  // glo_tow_acc
  // bds_tow_acc
  // qzss_tow_acc
  // num_sv
  // flags
  // sv_data
  if (!ublox_ubx_msgs__msg__MeasxData__Sequence__init(&msg->sv_data, 0)) {
    ublox_ubx_msgs__msg__UBXRxmMeasx__fini(msg);
    return false;
  }
  return true;
}

void
ublox_ubx_msgs__msg__UBXRxmMeasx__fini(ublox_ubx_msgs__msg__UBXRxmMeasx * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // version
  // gps_tow
  // glo_tow
  // bds_tow
  // qzss_tow
  // gps_tow_acc
  // glo_tow_acc
  // bds_tow_acc
  // qzss_tow_acc
  // num_sv
  // flags
  // sv_data
  ublox_ubx_msgs__msg__MeasxData__Sequence__fini(&msg->sv_data);
}

bool
ublox_ubx_msgs__msg__UBXRxmMeasx__are_equal(const ublox_ubx_msgs__msg__UBXRxmMeasx * lhs, const ublox_ubx_msgs__msg__UBXRxmMeasx * rhs)
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
  // gps_tow
  if (lhs->gps_tow != rhs->gps_tow) {
    return false;
  }
  // glo_tow
  if (lhs->glo_tow != rhs->glo_tow) {
    return false;
  }
  // bds_tow
  if (lhs->bds_tow != rhs->bds_tow) {
    return false;
  }
  // qzss_tow
  if (lhs->qzss_tow != rhs->qzss_tow) {
    return false;
  }
  // gps_tow_acc
  if (lhs->gps_tow_acc != rhs->gps_tow_acc) {
    return false;
  }
  // glo_tow_acc
  if (lhs->glo_tow_acc != rhs->glo_tow_acc) {
    return false;
  }
  // bds_tow_acc
  if (lhs->bds_tow_acc != rhs->bds_tow_acc) {
    return false;
  }
  // qzss_tow_acc
  if (lhs->qzss_tow_acc != rhs->qzss_tow_acc) {
    return false;
  }
  // num_sv
  if (lhs->num_sv != rhs->num_sv) {
    return false;
  }
  // flags
  if (lhs->flags != rhs->flags) {
    return false;
  }
  // sv_data
  if (!ublox_ubx_msgs__msg__MeasxData__Sequence__are_equal(
      &(lhs->sv_data), &(rhs->sv_data)))
  {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXRxmMeasx__copy(
  const ublox_ubx_msgs__msg__UBXRxmMeasx * input,
  ublox_ubx_msgs__msg__UBXRxmMeasx * output)
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
  // gps_tow
  output->gps_tow = input->gps_tow;
  // glo_tow
  output->glo_tow = input->glo_tow;
  // bds_tow
  output->bds_tow = input->bds_tow;
  // qzss_tow
  output->qzss_tow = input->qzss_tow;
  // gps_tow_acc
  output->gps_tow_acc = input->gps_tow_acc;
  // glo_tow_acc
  output->glo_tow_acc = input->glo_tow_acc;
  // bds_tow_acc
  output->bds_tow_acc = input->bds_tow_acc;
  // qzss_tow_acc
  output->qzss_tow_acc = input->qzss_tow_acc;
  // num_sv
  output->num_sv = input->num_sv;
  // flags
  output->flags = input->flags;
  // sv_data
  if (!ublox_ubx_msgs__msg__MeasxData__Sequence__copy(
      &(input->sv_data), &(output->sv_data)))
  {
    return false;
  }
  return true;
}

ublox_ubx_msgs__msg__UBXRxmMeasx *
ublox_ubx_msgs__msg__UBXRxmMeasx__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXRxmMeasx * msg = (ublox_ubx_msgs__msg__UBXRxmMeasx *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXRxmMeasx), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXRxmMeasx));
  bool success = ublox_ubx_msgs__msg__UBXRxmMeasx__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXRxmMeasx__destroy(ublox_ubx_msgs__msg__UBXRxmMeasx * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXRxmMeasx__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence__init(ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXRxmMeasx * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXRxmMeasx *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXRxmMeasx), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXRxmMeasx__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXRxmMeasx__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence__fini(ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXRxmMeasx__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence *
ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence * array = (ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence__destroy(ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence * lhs, const ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXRxmMeasx__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence * input,
  ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXRxmMeasx);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXRxmMeasx * data =
      (ublox_ubx_msgs__msg__UBXRxmMeasx *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXRxmMeasx__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXRxmMeasx__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXRxmMeasx__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
