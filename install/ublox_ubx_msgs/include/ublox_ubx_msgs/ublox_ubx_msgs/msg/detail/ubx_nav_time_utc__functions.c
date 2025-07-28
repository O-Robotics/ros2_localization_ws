// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavTimeUTC.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_time_utc__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `utc_std`
#include "ublox_ubx_msgs/msg/detail/utc_std__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavTimeUTC__init(ublox_ubx_msgs__msg__UBXNavTimeUTC * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavTimeUTC__fini(msg);
    return false;
  }
  // itow
  // t_acc
  // nano
  // year
  // month
  // day
  // hour
  // min
  // sec
  // valid_tow
  // valid_wkn
  // valid_utc
  // utc_std
  if (!ublox_ubx_msgs__msg__UtcStd__init(&msg->utc_std)) {
    ublox_ubx_msgs__msg__UBXNavTimeUTC__fini(msg);
    return false;
  }
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavTimeUTC__fini(ublox_ubx_msgs__msg__UBXNavTimeUTC * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // itow
  // t_acc
  // nano
  // year
  // month
  // day
  // hour
  // min
  // sec
  // valid_tow
  // valid_wkn
  // valid_utc
  // utc_std
  ublox_ubx_msgs__msg__UtcStd__fini(&msg->utc_std);
}

bool
ublox_ubx_msgs__msg__UBXNavTimeUTC__are_equal(const ublox_ubx_msgs__msg__UBXNavTimeUTC * lhs, const ublox_ubx_msgs__msg__UBXNavTimeUTC * rhs)
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
  // t_acc
  if (lhs->t_acc != rhs->t_acc) {
    return false;
  }
  // nano
  if (lhs->nano != rhs->nano) {
    return false;
  }
  // year
  if (lhs->year != rhs->year) {
    return false;
  }
  // month
  if (lhs->month != rhs->month) {
    return false;
  }
  // day
  if (lhs->day != rhs->day) {
    return false;
  }
  // hour
  if (lhs->hour != rhs->hour) {
    return false;
  }
  // min
  if (lhs->min != rhs->min) {
    return false;
  }
  // sec
  if (lhs->sec != rhs->sec) {
    return false;
  }
  // valid_tow
  if (lhs->valid_tow != rhs->valid_tow) {
    return false;
  }
  // valid_wkn
  if (lhs->valid_wkn != rhs->valid_wkn) {
    return false;
  }
  // valid_utc
  if (lhs->valid_utc != rhs->valid_utc) {
    return false;
  }
  // utc_std
  if (!ublox_ubx_msgs__msg__UtcStd__are_equal(
      &(lhs->utc_std), &(rhs->utc_std)))
  {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavTimeUTC__copy(
  const ublox_ubx_msgs__msg__UBXNavTimeUTC * input,
  ublox_ubx_msgs__msg__UBXNavTimeUTC * output)
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
  // t_acc
  output->t_acc = input->t_acc;
  // nano
  output->nano = input->nano;
  // year
  output->year = input->year;
  // month
  output->month = input->month;
  // day
  output->day = input->day;
  // hour
  output->hour = input->hour;
  // min
  output->min = input->min;
  // sec
  output->sec = input->sec;
  // valid_tow
  output->valid_tow = input->valid_tow;
  // valid_wkn
  output->valid_wkn = input->valid_wkn;
  // valid_utc
  output->valid_utc = input->valid_utc;
  // utc_std
  if (!ublox_ubx_msgs__msg__UtcStd__copy(
      &(input->utc_std), &(output->utc_std)))
  {
    return false;
  }
  return true;
}

ublox_ubx_msgs__msg__UBXNavTimeUTC *
ublox_ubx_msgs__msg__UBXNavTimeUTC__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavTimeUTC * msg = (ublox_ubx_msgs__msg__UBXNavTimeUTC *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavTimeUTC), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavTimeUTC));
  bool success = ublox_ubx_msgs__msg__UBXNavTimeUTC__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavTimeUTC__destroy(ublox_ubx_msgs__msg__UBXNavTimeUTC * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavTimeUTC__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence__init(ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavTimeUTC * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavTimeUTC *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavTimeUTC), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavTimeUTC__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavTimeUTC__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence__fini(ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavTimeUTC__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence *
ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence * array = (ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavTimeUTC__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavTimeUTC);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavTimeUTC * data =
      (ublox_ubx_msgs__msg__UBXNavTimeUTC *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavTimeUTC__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavTimeUTC__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavTimeUTC__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
