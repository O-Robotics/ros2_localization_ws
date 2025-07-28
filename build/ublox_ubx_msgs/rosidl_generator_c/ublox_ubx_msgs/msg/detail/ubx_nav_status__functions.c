// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavStatus.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `gps_fix`
#include "ublox_ubx_msgs/msg/detail/gps_fix__functions.h"
// Member `map_matching`
#include "ublox_ubx_msgs/msg/detail/map_matching__functions.h"
// Member `psm`
#include "ublox_ubx_msgs/msg/detail/psm_status__functions.h"
// Member `spoof_det`
#include "ublox_ubx_msgs/msg/detail/spoof_det__functions.h"
// Member `carr_soln`
#include "ublox_ubx_msgs/msg/detail/carr_soln__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavStatus__init(ublox_ubx_msgs__msg__UBXNavStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavStatus__fini(msg);
    return false;
  }
  // itow
  // gps_fix
  if (!ublox_ubx_msgs__msg__GpsFix__init(&msg->gps_fix)) {
    ublox_ubx_msgs__msg__UBXNavStatus__fini(msg);
    return false;
  }
  // gps_fix_ok
  // diff_soln
  // wkn_set
  // tow_set
  // diff_corr
  // carr_soln_valid
  // map_matching
  if (!ublox_ubx_msgs__msg__MapMatching__init(&msg->map_matching)) {
    ublox_ubx_msgs__msg__UBXNavStatus__fini(msg);
    return false;
  }
  // psm
  if (!ublox_ubx_msgs__msg__PSMStatus__init(&msg->psm)) {
    ublox_ubx_msgs__msg__UBXNavStatus__fini(msg);
    return false;
  }
  // spoof_det
  if (!ublox_ubx_msgs__msg__SpoofDet__init(&msg->spoof_det)) {
    ublox_ubx_msgs__msg__UBXNavStatus__fini(msg);
    return false;
  }
  // carr_soln
  if (!ublox_ubx_msgs__msg__CarrSoln__init(&msg->carr_soln)) {
    ublox_ubx_msgs__msg__UBXNavStatus__fini(msg);
    return false;
  }
  // ttff
  // msss
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavStatus__fini(ublox_ubx_msgs__msg__UBXNavStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // itow
  // gps_fix
  ublox_ubx_msgs__msg__GpsFix__fini(&msg->gps_fix);
  // gps_fix_ok
  // diff_soln
  // wkn_set
  // tow_set
  // diff_corr
  // carr_soln_valid
  // map_matching
  ublox_ubx_msgs__msg__MapMatching__fini(&msg->map_matching);
  // psm
  ublox_ubx_msgs__msg__PSMStatus__fini(&msg->psm);
  // spoof_det
  ublox_ubx_msgs__msg__SpoofDet__fini(&msg->spoof_det);
  // carr_soln
  ublox_ubx_msgs__msg__CarrSoln__fini(&msg->carr_soln);
  // ttff
  // msss
}

bool
ublox_ubx_msgs__msg__UBXNavStatus__are_equal(const ublox_ubx_msgs__msg__UBXNavStatus * lhs, const ublox_ubx_msgs__msg__UBXNavStatus * rhs)
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
  // gps_fix
  if (!ublox_ubx_msgs__msg__GpsFix__are_equal(
      &(lhs->gps_fix), &(rhs->gps_fix)))
  {
    return false;
  }
  // gps_fix_ok
  if (lhs->gps_fix_ok != rhs->gps_fix_ok) {
    return false;
  }
  // diff_soln
  if (lhs->diff_soln != rhs->diff_soln) {
    return false;
  }
  // wkn_set
  if (lhs->wkn_set != rhs->wkn_set) {
    return false;
  }
  // tow_set
  if (lhs->tow_set != rhs->tow_set) {
    return false;
  }
  // diff_corr
  if (lhs->diff_corr != rhs->diff_corr) {
    return false;
  }
  // carr_soln_valid
  if (lhs->carr_soln_valid != rhs->carr_soln_valid) {
    return false;
  }
  // map_matching
  if (!ublox_ubx_msgs__msg__MapMatching__are_equal(
      &(lhs->map_matching), &(rhs->map_matching)))
  {
    return false;
  }
  // psm
  if (!ublox_ubx_msgs__msg__PSMStatus__are_equal(
      &(lhs->psm), &(rhs->psm)))
  {
    return false;
  }
  // spoof_det
  if (!ublox_ubx_msgs__msg__SpoofDet__are_equal(
      &(lhs->spoof_det), &(rhs->spoof_det)))
  {
    return false;
  }
  // carr_soln
  if (!ublox_ubx_msgs__msg__CarrSoln__are_equal(
      &(lhs->carr_soln), &(rhs->carr_soln)))
  {
    return false;
  }
  // ttff
  if (lhs->ttff != rhs->ttff) {
    return false;
  }
  // msss
  if (lhs->msss != rhs->msss) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavStatus__copy(
  const ublox_ubx_msgs__msg__UBXNavStatus * input,
  ublox_ubx_msgs__msg__UBXNavStatus * output)
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
  // gps_fix
  if (!ublox_ubx_msgs__msg__GpsFix__copy(
      &(input->gps_fix), &(output->gps_fix)))
  {
    return false;
  }
  // gps_fix_ok
  output->gps_fix_ok = input->gps_fix_ok;
  // diff_soln
  output->diff_soln = input->diff_soln;
  // wkn_set
  output->wkn_set = input->wkn_set;
  // tow_set
  output->tow_set = input->tow_set;
  // diff_corr
  output->diff_corr = input->diff_corr;
  // carr_soln_valid
  output->carr_soln_valid = input->carr_soln_valid;
  // map_matching
  if (!ublox_ubx_msgs__msg__MapMatching__copy(
      &(input->map_matching), &(output->map_matching)))
  {
    return false;
  }
  // psm
  if (!ublox_ubx_msgs__msg__PSMStatus__copy(
      &(input->psm), &(output->psm)))
  {
    return false;
  }
  // spoof_det
  if (!ublox_ubx_msgs__msg__SpoofDet__copy(
      &(input->spoof_det), &(output->spoof_det)))
  {
    return false;
  }
  // carr_soln
  if (!ublox_ubx_msgs__msg__CarrSoln__copy(
      &(input->carr_soln), &(output->carr_soln)))
  {
    return false;
  }
  // ttff
  output->ttff = input->ttff;
  // msss
  output->msss = input->msss;
  return true;
}

ublox_ubx_msgs__msg__UBXNavStatus *
ublox_ubx_msgs__msg__UBXNavStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavStatus * msg = (ublox_ubx_msgs__msg__UBXNavStatus *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavStatus));
  bool success = ublox_ubx_msgs__msg__UBXNavStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavStatus__destroy(ublox_ubx_msgs__msg__UBXNavStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavStatus__Sequence__init(ublox_ubx_msgs__msg__UBXNavStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavStatus * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavStatus *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavStatus__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavStatus__Sequence__fini(ublox_ubx_msgs__msg__UBXNavStatus__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavStatus__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavStatus__Sequence *
ublox_ubx_msgs__msg__UBXNavStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavStatus__Sequence * array = (ublox_ubx_msgs__msg__UBXNavStatus__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavStatus__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavStatus__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavStatus__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavStatus__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavStatus__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavStatus * data =
      (ublox_ubx_msgs__msg__UBXNavStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
