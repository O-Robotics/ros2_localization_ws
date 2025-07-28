// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/MeasxData.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/measx_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ublox_ubx_msgs__msg__MeasxData__init(ublox_ubx_msgs__msg__MeasxData * msg)
{
  if (!msg) {
    return false;
  }
  // gnss_id
  // sv_id
  // c_no
  // mpath_indic
  // doppler_ms
  // doppler_hz
  // whole_chips
  // frac_chips
  // code_phase
  // int_code_phase
  // pseu_range_rms_err
  return true;
}

void
ublox_ubx_msgs__msg__MeasxData__fini(ublox_ubx_msgs__msg__MeasxData * msg)
{
  if (!msg) {
    return;
  }
  // gnss_id
  // sv_id
  // c_no
  // mpath_indic
  // doppler_ms
  // doppler_hz
  // whole_chips
  // frac_chips
  // code_phase
  // int_code_phase
  // pseu_range_rms_err
}

bool
ublox_ubx_msgs__msg__MeasxData__are_equal(const ublox_ubx_msgs__msg__MeasxData * lhs, const ublox_ubx_msgs__msg__MeasxData * rhs)
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
  // c_no
  if (lhs->c_no != rhs->c_no) {
    return false;
  }
  // mpath_indic
  if (lhs->mpath_indic != rhs->mpath_indic) {
    return false;
  }
  // doppler_ms
  if (lhs->doppler_ms != rhs->doppler_ms) {
    return false;
  }
  // doppler_hz
  if (lhs->doppler_hz != rhs->doppler_hz) {
    return false;
  }
  // whole_chips
  if (lhs->whole_chips != rhs->whole_chips) {
    return false;
  }
  // frac_chips
  if (lhs->frac_chips != rhs->frac_chips) {
    return false;
  }
  // code_phase
  if (lhs->code_phase != rhs->code_phase) {
    return false;
  }
  // int_code_phase
  if (lhs->int_code_phase != rhs->int_code_phase) {
    return false;
  }
  // pseu_range_rms_err
  if (lhs->pseu_range_rms_err != rhs->pseu_range_rms_err) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__MeasxData__copy(
  const ublox_ubx_msgs__msg__MeasxData * input,
  ublox_ubx_msgs__msg__MeasxData * output)
{
  if (!input || !output) {
    return false;
  }
  // gnss_id
  output->gnss_id = input->gnss_id;
  // sv_id
  output->sv_id = input->sv_id;
  // c_no
  output->c_no = input->c_no;
  // mpath_indic
  output->mpath_indic = input->mpath_indic;
  // doppler_ms
  output->doppler_ms = input->doppler_ms;
  // doppler_hz
  output->doppler_hz = input->doppler_hz;
  // whole_chips
  output->whole_chips = input->whole_chips;
  // frac_chips
  output->frac_chips = input->frac_chips;
  // code_phase
  output->code_phase = input->code_phase;
  // int_code_phase
  output->int_code_phase = input->int_code_phase;
  // pseu_range_rms_err
  output->pseu_range_rms_err = input->pseu_range_rms_err;
  return true;
}

ublox_ubx_msgs__msg__MeasxData *
ublox_ubx_msgs__msg__MeasxData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__MeasxData * msg = (ublox_ubx_msgs__msg__MeasxData *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__MeasxData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__MeasxData));
  bool success = ublox_ubx_msgs__msg__MeasxData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__MeasxData__destroy(ublox_ubx_msgs__msg__MeasxData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__MeasxData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__MeasxData__Sequence__init(ublox_ubx_msgs__msg__MeasxData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__MeasxData * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__MeasxData *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__MeasxData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__MeasxData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__MeasxData__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__MeasxData__Sequence__fini(ublox_ubx_msgs__msg__MeasxData__Sequence * array)
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
      ublox_ubx_msgs__msg__MeasxData__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__MeasxData__Sequence *
ublox_ubx_msgs__msg__MeasxData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__MeasxData__Sequence * array = (ublox_ubx_msgs__msg__MeasxData__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__MeasxData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__MeasxData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__MeasxData__Sequence__destroy(ublox_ubx_msgs__msg__MeasxData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__MeasxData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__MeasxData__Sequence__are_equal(const ublox_ubx_msgs__msg__MeasxData__Sequence * lhs, const ublox_ubx_msgs__msg__MeasxData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__MeasxData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__MeasxData__Sequence__copy(
  const ublox_ubx_msgs__msg__MeasxData__Sequence * input,
  ublox_ubx_msgs__msg__MeasxData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__MeasxData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__MeasxData * data =
      (ublox_ubx_msgs__msg__MeasxData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__MeasxData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__MeasxData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__MeasxData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
