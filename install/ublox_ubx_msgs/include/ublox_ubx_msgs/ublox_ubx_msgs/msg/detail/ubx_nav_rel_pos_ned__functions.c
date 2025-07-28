// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavRelPosNED.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `carr_soln`
#include "ublox_ubx_msgs/msg/detail/carr_soln__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavRelPosNED__init(ublox_ubx_msgs__msg__UBXNavRelPosNED * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavRelPosNED__fini(msg);
    return false;
  }
  // version
  // ref_station_id
  // itow
  // rel_pos_n
  // rel_pos_e
  // rel_pos_d
  // rel_pos_length
  // rel_pos_heading
  // rel_pos_hp_n
  // rel_pos_hp_e
  // rel_pos_hp_d
  // rel_pos_hp_length
  // acc_n
  // acc_e
  // acc_d
  // acc_length
  // acc_heading
  // gnss_fix_ok
  // diff_soln
  // rel_pos_valid
  // carr_soln
  if (!ublox_ubx_msgs__msg__CarrSoln__init(&msg->carr_soln)) {
    ublox_ubx_msgs__msg__UBXNavRelPosNED__fini(msg);
    return false;
  }
  // is_moving
  // ref_pos_miss
  // ref_obs_miss
  // rel_pos_heading_valid
  // rel_pos_normalized
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavRelPosNED__fini(ublox_ubx_msgs__msg__UBXNavRelPosNED * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // version
  // ref_station_id
  // itow
  // rel_pos_n
  // rel_pos_e
  // rel_pos_d
  // rel_pos_length
  // rel_pos_heading
  // rel_pos_hp_n
  // rel_pos_hp_e
  // rel_pos_hp_d
  // rel_pos_hp_length
  // acc_n
  // acc_e
  // acc_d
  // acc_length
  // acc_heading
  // gnss_fix_ok
  // diff_soln
  // rel_pos_valid
  // carr_soln
  ublox_ubx_msgs__msg__CarrSoln__fini(&msg->carr_soln);
  // is_moving
  // ref_pos_miss
  // ref_obs_miss
  // rel_pos_heading_valid
  // rel_pos_normalized
}

bool
ublox_ubx_msgs__msg__UBXNavRelPosNED__are_equal(const ublox_ubx_msgs__msg__UBXNavRelPosNED * lhs, const ublox_ubx_msgs__msg__UBXNavRelPosNED * rhs)
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
  // ref_station_id
  if (lhs->ref_station_id != rhs->ref_station_id) {
    return false;
  }
  // itow
  if (lhs->itow != rhs->itow) {
    return false;
  }
  // rel_pos_n
  if (lhs->rel_pos_n != rhs->rel_pos_n) {
    return false;
  }
  // rel_pos_e
  if (lhs->rel_pos_e != rhs->rel_pos_e) {
    return false;
  }
  // rel_pos_d
  if (lhs->rel_pos_d != rhs->rel_pos_d) {
    return false;
  }
  // rel_pos_length
  if (lhs->rel_pos_length != rhs->rel_pos_length) {
    return false;
  }
  // rel_pos_heading
  if (lhs->rel_pos_heading != rhs->rel_pos_heading) {
    return false;
  }
  // rel_pos_hp_n
  if (lhs->rel_pos_hp_n != rhs->rel_pos_hp_n) {
    return false;
  }
  // rel_pos_hp_e
  if (lhs->rel_pos_hp_e != rhs->rel_pos_hp_e) {
    return false;
  }
  // rel_pos_hp_d
  if (lhs->rel_pos_hp_d != rhs->rel_pos_hp_d) {
    return false;
  }
  // rel_pos_hp_length
  if (lhs->rel_pos_hp_length != rhs->rel_pos_hp_length) {
    return false;
  }
  // acc_n
  if (lhs->acc_n != rhs->acc_n) {
    return false;
  }
  // acc_e
  if (lhs->acc_e != rhs->acc_e) {
    return false;
  }
  // acc_d
  if (lhs->acc_d != rhs->acc_d) {
    return false;
  }
  // acc_length
  if (lhs->acc_length != rhs->acc_length) {
    return false;
  }
  // acc_heading
  if (lhs->acc_heading != rhs->acc_heading) {
    return false;
  }
  // gnss_fix_ok
  if (lhs->gnss_fix_ok != rhs->gnss_fix_ok) {
    return false;
  }
  // diff_soln
  if (lhs->diff_soln != rhs->diff_soln) {
    return false;
  }
  // rel_pos_valid
  if (lhs->rel_pos_valid != rhs->rel_pos_valid) {
    return false;
  }
  // carr_soln
  if (!ublox_ubx_msgs__msg__CarrSoln__are_equal(
      &(lhs->carr_soln), &(rhs->carr_soln)))
  {
    return false;
  }
  // is_moving
  if (lhs->is_moving != rhs->is_moving) {
    return false;
  }
  // ref_pos_miss
  if (lhs->ref_pos_miss != rhs->ref_pos_miss) {
    return false;
  }
  // ref_obs_miss
  if (lhs->ref_obs_miss != rhs->ref_obs_miss) {
    return false;
  }
  // rel_pos_heading_valid
  if (lhs->rel_pos_heading_valid != rhs->rel_pos_heading_valid) {
    return false;
  }
  // rel_pos_normalized
  if (lhs->rel_pos_normalized != rhs->rel_pos_normalized) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavRelPosNED__copy(
  const ublox_ubx_msgs__msg__UBXNavRelPosNED * input,
  ublox_ubx_msgs__msg__UBXNavRelPosNED * output)
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
  // ref_station_id
  output->ref_station_id = input->ref_station_id;
  // itow
  output->itow = input->itow;
  // rel_pos_n
  output->rel_pos_n = input->rel_pos_n;
  // rel_pos_e
  output->rel_pos_e = input->rel_pos_e;
  // rel_pos_d
  output->rel_pos_d = input->rel_pos_d;
  // rel_pos_length
  output->rel_pos_length = input->rel_pos_length;
  // rel_pos_heading
  output->rel_pos_heading = input->rel_pos_heading;
  // rel_pos_hp_n
  output->rel_pos_hp_n = input->rel_pos_hp_n;
  // rel_pos_hp_e
  output->rel_pos_hp_e = input->rel_pos_hp_e;
  // rel_pos_hp_d
  output->rel_pos_hp_d = input->rel_pos_hp_d;
  // rel_pos_hp_length
  output->rel_pos_hp_length = input->rel_pos_hp_length;
  // acc_n
  output->acc_n = input->acc_n;
  // acc_e
  output->acc_e = input->acc_e;
  // acc_d
  output->acc_d = input->acc_d;
  // acc_length
  output->acc_length = input->acc_length;
  // acc_heading
  output->acc_heading = input->acc_heading;
  // gnss_fix_ok
  output->gnss_fix_ok = input->gnss_fix_ok;
  // diff_soln
  output->diff_soln = input->diff_soln;
  // rel_pos_valid
  output->rel_pos_valid = input->rel_pos_valid;
  // carr_soln
  if (!ublox_ubx_msgs__msg__CarrSoln__copy(
      &(input->carr_soln), &(output->carr_soln)))
  {
    return false;
  }
  // is_moving
  output->is_moving = input->is_moving;
  // ref_pos_miss
  output->ref_pos_miss = input->ref_pos_miss;
  // ref_obs_miss
  output->ref_obs_miss = input->ref_obs_miss;
  // rel_pos_heading_valid
  output->rel_pos_heading_valid = input->rel_pos_heading_valid;
  // rel_pos_normalized
  output->rel_pos_normalized = input->rel_pos_normalized;
  return true;
}

ublox_ubx_msgs__msg__UBXNavRelPosNED *
ublox_ubx_msgs__msg__UBXNavRelPosNED__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavRelPosNED * msg = (ublox_ubx_msgs__msg__UBXNavRelPosNED *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavRelPosNED), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavRelPosNED));
  bool success = ublox_ubx_msgs__msg__UBXNavRelPosNED__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavRelPosNED__destroy(ublox_ubx_msgs__msg__UBXNavRelPosNED * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavRelPosNED__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence__init(ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavRelPosNED * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavRelPosNED *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavRelPosNED), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavRelPosNED__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavRelPosNED__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence__fini(ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavRelPosNED__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence *
ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence * array = (ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavRelPosNED__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavRelPosNED);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavRelPosNED * data =
      (ublox_ubx_msgs__msg__UBXNavRelPosNED *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavRelPosNED__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavRelPosNED__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavRelPosNED__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
