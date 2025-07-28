// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/ESFSensorStatus.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ublox_ubx_msgs__msg__ESFSensorStatus__init(ublox_ubx_msgs__msg__ESFSensorStatus * msg)
{
  if (!msg) {
    return false;
  }
  // sensor_data_type
  // used
  // ready
  // calib_status
  // time_status
  // freq
  // fault_bad_meas
  // fault_bad_ttag
  // fault_missing_meas
  // fault_noisy_meas
  return true;
}

void
ublox_ubx_msgs__msg__ESFSensorStatus__fini(ublox_ubx_msgs__msg__ESFSensorStatus * msg)
{
  if (!msg) {
    return;
  }
  // sensor_data_type
  // used
  // ready
  // calib_status
  // time_status
  // freq
  // fault_bad_meas
  // fault_bad_ttag
  // fault_missing_meas
  // fault_noisy_meas
}

bool
ublox_ubx_msgs__msg__ESFSensorStatus__are_equal(const ublox_ubx_msgs__msg__ESFSensorStatus * lhs, const ublox_ubx_msgs__msg__ESFSensorStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sensor_data_type
  if (lhs->sensor_data_type != rhs->sensor_data_type) {
    return false;
  }
  // used
  if (lhs->used != rhs->used) {
    return false;
  }
  // ready
  if (lhs->ready != rhs->ready) {
    return false;
  }
  // calib_status
  if (lhs->calib_status != rhs->calib_status) {
    return false;
  }
  // time_status
  if (lhs->time_status != rhs->time_status) {
    return false;
  }
  // freq
  if (lhs->freq != rhs->freq) {
    return false;
  }
  // fault_bad_meas
  if (lhs->fault_bad_meas != rhs->fault_bad_meas) {
    return false;
  }
  // fault_bad_ttag
  if (lhs->fault_bad_ttag != rhs->fault_bad_ttag) {
    return false;
  }
  // fault_missing_meas
  if (lhs->fault_missing_meas != rhs->fault_missing_meas) {
    return false;
  }
  // fault_noisy_meas
  if (lhs->fault_noisy_meas != rhs->fault_noisy_meas) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__ESFSensorStatus__copy(
  const ublox_ubx_msgs__msg__ESFSensorStatus * input,
  ublox_ubx_msgs__msg__ESFSensorStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // sensor_data_type
  output->sensor_data_type = input->sensor_data_type;
  // used
  output->used = input->used;
  // ready
  output->ready = input->ready;
  // calib_status
  output->calib_status = input->calib_status;
  // time_status
  output->time_status = input->time_status;
  // freq
  output->freq = input->freq;
  // fault_bad_meas
  output->fault_bad_meas = input->fault_bad_meas;
  // fault_bad_ttag
  output->fault_bad_ttag = input->fault_bad_ttag;
  // fault_missing_meas
  output->fault_missing_meas = input->fault_missing_meas;
  // fault_noisy_meas
  output->fault_noisy_meas = input->fault_noisy_meas;
  return true;
}

ublox_ubx_msgs__msg__ESFSensorStatus *
ublox_ubx_msgs__msg__ESFSensorStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__ESFSensorStatus * msg = (ublox_ubx_msgs__msg__ESFSensorStatus *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__ESFSensorStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__ESFSensorStatus));
  bool success = ublox_ubx_msgs__msg__ESFSensorStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__ESFSensorStatus__destroy(ublox_ubx_msgs__msg__ESFSensorStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__ESFSensorStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__init(ublox_ubx_msgs__msg__ESFSensorStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__ESFSensorStatus * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__ESFSensorStatus *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__ESFSensorStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__ESFSensorStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__ESFSensorStatus__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__fini(ublox_ubx_msgs__msg__ESFSensorStatus__Sequence * array)
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
      ublox_ubx_msgs__msg__ESFSensorStatus__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__ESFSensorStatus__Sequence *
ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__ESFSensorStatus__Sequence * array = (ublox_ubx_msgs__msg__ESFSensorStatus__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__ESFSensorStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__destroy(ublox_ubx_msgs__msg__ESFSensorStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__are_equal(const ublox_ubx_msgs__msg__ESFSensorStatus__Sequence * lhs, const ublox_ubx_msgs__msg__ESFSensorStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__ESFSensorStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__copy(
  const ublox_ubx_msgs__msg__ESFSensorStatus__Sequence * input,
  ublox_ubx_msgs__msg__ESFSensorStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__ESFSensorStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__ESFSensorStatus * data =
      (ublox_ubx_msgs__msg__ESFSensorStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__ESFSensorStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__ESFSensorStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__ESFSensorStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
