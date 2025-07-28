// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/SatFlags.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sat_flags__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ublox_ubx_msgs__msg__SatFlags__init(ublox_ubx_msgs__msg__SatFlags * msg)
{
  if (!msg) {
    return false;
  }
  // quality_ind
  // sv_used
  // health
  // diff_corr
  // smoothed
  // orbit_source
  // eph_avail
  // alm_avail
  // ano_avail
  // aop_avail
  // sbas_corr_used
  // rtcm_corr_used
  // slas_corr_used
  // spartn_corr_used
  // pr_corr_used
  // cr_corr_used
  // do_corr_used
  // clas_corr_used
  return true;
}

void
ublox_ubx_msgs__msg__SatFlags__fini(ublox_ubx_msgs__msg__SatFlags * msg)
{
  if (!msg) {
    return;
  }
  // quality_ind
  // sv_used
  // health
  // diff_corr
  // smoothed
  // orbit_source
  // eph_avail
  // alm_avail
  // ano_avail
  // aop_avail
  // sbas_corr_used
  // rtcm_corr_used
  // slas_corr_used
  // spartn_corr_used
  // pr_corr_used
  // cr_corr_used
  // do_corr_used
  // clas_corr_used
}

bool
ublox_ubx_msgs__msg__SatFlags__are_equal(const ublox_ubx_msgs__msg__SatFlags * lhs, const ublox_ubx_msgs__msg__SatFlags * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // quality_ind
  if (lhs->quality_ind != rhs->quality_ind) {
    return false;
  }
  // sv_used
  if (lhs->sv_used != rhs->sv_used) {
    return false;
  }
  // health
  if (lhs->health != rhs->health) {
    return false;
  }
  // diff_corr
  if (lhs->diff_corr != rhs->diff_corr) {
    return false;
  }
  // smoothed
  if (lhs->smoothed != rhs->smoothed) {
    return false;
  }
  // orbit_source
  if (lhs->orbit_source != rhs->orbit_source) {
    return false;
  }
  // eph_avail
  if (lhs->eph_avail != rhs->eph_avail) {
    return false;
  }
  // alm_avail
  if (lhs->alm_avail != rhs->alm_avail) {
    return false;
  }
  // ano_avail
  if (lhs->ano_avail != rhs->ano_avail) {
    return false;
  }
  // aop_avail
  if (lhs->aop_avail != rhs->aop_avail) {
    return false;
  }
  // sbas_corr_used
  if (lhs->sbas_corr_used != rhs->sbas_corr_used) {
    return false;
  }
  // rtcm_corr_used
  if (lhs->rtcm_corr_used != rhs->rtcm_corr_used) {
    return false;
  }
  // slas_corr_used
  if (lhs->slas_corr_used != rhs->slas_corr_used) {
    return false;
  }
  // spartn_corr_used
  if (lhs->spartn_corr_used != rhs->spartn_corr_used) {
    return false;
  }
  // pr_corr_used
  if (lhs->pr_corr_used != rhs->pr_corr_used) {
    return false;
  }
  // cr_corr_used
  if (lhs->cr_corr_used != rhs->cr_corr_used) {
    return false;
  }
  // do_corr_used
  if (lhs->do_corr_used != rhs->do_corr_used) {
    return false;
  }
  // clas_corr_used
  if (lhs->clas_corr_used != rhs->clas_corr_used) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SatFlags__copy(
  const ublox_ubx_msgs__msg__SatFlags * input,
  ublox_ubx_msgs__msg__SatFlags * output)
{
  if (!input || !output) {
    return false;
  }
  // quality_ind
  output->quality_ind = input->quality_ind;
  // sv_used
  output->sv_used = input->sv_used;
  // health
  output->health = input->health;
  // diff_corr
  output->diff_corr = input->diff_corr;
  // smoothed
  output->smoothed = input->smoothed;
  // orbit_source
  output->orbit_source = input->orbit_source;
  // eph_avail
  output->eph_avail = input->eph_avail;
  // alm_avail
  output->alm_avail = input->alm_avail;
  // ano_avail
  output->ano_avail = input->ano_avail;
  // aop_avail
  output->aop_avail = input->aop_avail;
  // sbas_corr_used
  output->sbas_corr_used = input->sbas_corr_used;
  // rtcm_corr_used
  output->rtcm_corr_used = input->rtcm_corr_used;
  // slas_corr_used
  output->slas_corr_used = input->slas_corr_used;
  // spartn_corr_used
  output->spartn_corr_used = input->spartn_corr_used;
  // pr_corr_used
  output->pr_corr_used = input->pr_corr_used;
  // cr_corr_used
  output->cr_corr_used = input->cr_corr_used;
  // do_corr_used
  output->do_corr_used = input->do_corr_used;
  // clas_corr_used
  output->clas_corr_used = input->clas_corr_used;
  return true;
}

ublox_ubx_msgs__msg__SatFlags *
ublox_ubx_msgs__msg__SatFlags__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SatFlags * msg = (ublox_ubx_msgs__msg__SatFlags *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SatFlags), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__SatFlags));
  bool success = ublox_ubx_msgs__msg__SatFlags__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__SatFlags__destroy(ublox_ubx_msgs__msg__SatFlags * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__SatFlags__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__SatFlags__Sequence__init(ublox_ubx_msgs__msg__SatFlags__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SatFlags * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__SatFlags *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__SatFlags), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__SatFlags__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__SatFlags__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__SatFlags__Sequence__fini(ublox_ubx_msgs__msg__SatFlags__Sequence * array)
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
      ublox_ubx_msgs__msg__SatFlags__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__SatFlags__Sequence *
ublox_ubx_msgs__msg__SatFlags__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__SatFlags__Sequence * array = (ublox_ubx_msgs__msg__SatFlags__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__SatFlags__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__SatFlags__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__SatFlags__Sequence__destroy(ublox_ubx_msgs__msg__SatFlags__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__SatFlags__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__SatFlags__Sequence__are_equal(const ublox_ubx_msgs__msg__SatFlags__Sequence * lhs, const ublox_ubx_msgs__msg__SatFlags__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__SatFlags__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__SatFlags__Sequence__copy(
  const ublox_ubx_msgs__msg__SatFlags__Sequence * input,
  ublox_ubx_msgs__msg__SatFlags__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__SatFlags);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__SatFlags * data =
      (ublox_ubx_msgs__msg__SatFlags *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__SatFlags__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__SatFlags__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__SatFlags__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
