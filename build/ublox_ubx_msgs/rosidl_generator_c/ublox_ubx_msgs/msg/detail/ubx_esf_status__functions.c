// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXEsfStatus.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_esf_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sensor_statuses`
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__functions.h"

bool
ublox_ubx_msgs__msg__UBXEsfStatus__init(ublox_ubx_msgs__msg__UBXEsfStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXEsfStatus__fini(msg);
    return false;
  }
  // itow
  // version
  // wt_init_status
  // mnt_alg_status
  // ins_init_status
  // imu_init_status
  // fusion_mode
  // num_sens
  // sensor_statuses
  if (!ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__init(&msg->sensor_statuses, 0)) {
    ublox_ubx_msgs__msg__UBXEsfStatus__fini(msg);
    return false;
  }
  return true;
}

void
ublox_ubx_msgs__msg__UBXEsfStatus__fini(ublox_ubx_msgs__msg__UBXEsfStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // itow
  // version
  // wt_init_status
  // mnt_alg_status
  // ins_init_status
  // imu_init_status
  // fusion_mode
  // num_sens
  // sensor_statuses
  ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__fini(&msg->sensor_statuses);
}

bool
ublox_ubx_msgs__msg__UBXEsfStatus__are_equal(const ublox_ubx_msgs__msg__UBXEsfStatus * lhs, const ublox_ubx_msgs__msg__UBXEsfStatus * rhs)
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
  // wt_init_status
  if (lhs->wt_init_status != rhs->wt_init_status) {
    return false;
  }
  // mnt_alg_status
  if (lhs->mnt_alg_status != rhs->mnt_alg_status) {
    return false;
  }
  // ins_init_status
  if (lhs->ins_init_status != rhs->ins_init_status) {
    return false;
  }
  // imu_init_status
  if (lhs->imu_init_status != rhs->imu_init_status) {
    return false;
  }
  // fusion_mode
  if (lhs->fusion_mode != rhs->fusion_mode) {
    return false;
  }
  // num_sens
  if (lhs->num_sens != rhs->num_sens) {
    return false;
  }
  // sensor_statuses
  if (!ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__are_equal(
      &(lhs->sensor_statuses), &(rhs->sensor_statuses)))
  {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXEsfStatus__copy(
  const ublox_ubx_msgs__msg__UBXEsfStatus * input,
  ublox_ubx_msgs__msg__UBXEsfStatus * output)
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
  // wt_init_status
  output->wt_init_status = input->wt_init_status;
  // mnt_alg_status
  output->mnt_alg_status = input->mnt_alg_status;
  // ins_init_status
  output->ins_init_status = input->ins_init_status;
  // imu_init_status
  output->imu_init_status = input->imu_init_status;
  // fusion_mode
  output->fusion_mode = input->fusion_mode;
  // num_sens
  output->num_sens = input->num_sens;
  // sensor_statuses
  if (!ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__copy(
      &(input->sensor_statuses), &(output->sensor_statuses)))
  {
    return false;
  }
  return true;
}

ublox_ubx_msgs__msg__UBXEsfStatus *
ublox_ubx_msgs__msg__UBXEsfStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXEsfStatus * msg = (ublox_ubx_msgs__msg__UBXEsfStatus *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXEsfStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXEsfStatus));
  bool success = ublox_ubx_msgs__msg__UBXEsfStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXEsfStatus__destroy(ublox_ubx_msgs__msg__UBXEsfStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXEsfStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXEsfStatus__Sequence__init(ublox_ubx_msgs__msg__UBXEsfStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXEsfStatus * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXEsfStatus *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXEsfStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXEsfStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXEsfStatus__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXEsfStatus__Sequence__fini(ublox_ubx_msgs__msg__UBXEsfStatus__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXEsfStatus__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXEsfStatus__Sequence *
ublox_ubx_msgs__msg__UBXEsfStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXEsfStatus__Sequence * array = (ublox_ubx_msgs__msg__UBXEsfStatus__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXEsfStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXEsfStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXEsfStatus__Sequence__destroy(ublox_ubx_msgs__msg__UBXEsfStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXEsfStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXEsfStatus__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXEsfStatus__Sequence * lhs, const ublox_ubx_msgs__msg__UBXEsfStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXEsfStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXEsfStatus__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXEsfStatus__Sequence * input,
  ublox_ubx_msgs__msg__UBXEsfStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXEsfStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXEsfStatus * data =
      (ublox_ubx_msgs__msg__UBXEsfStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXEsfStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXEsfStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXEsfStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
