// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_interfaces:srv/ColdStart.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_interfaces/srv/detail/cold_start__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
ublox_ubx_interfaces__srv__ColdStart_Request__init(ublox_ubx_interfaces__srv__ColdStart_Request * msg)
{
  if (!msg) {
    return false;
  }
  // reset_type
  return true;
}

void
ublox_ubx_interfaces__srv__ColdStart_Request__fini(ublox_ubx_interfaces__srv__ColdStart_Request * msg)
{
  if (!msg) {
    return;
  }
  // reset_type
}

bool
ublox_ubx_interfaces__srv__ColdStart_Request__are_equal(const ublox_ubx_interfaces__srv__ColdStart_Request * lhs, const ublox_ubx_interfaces__srv__ColdStart_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // reset_type
  if (lhs->reset_type != rhs->reset_type) {
    return false;
  }
  return true;
}

bool
ublox_ubx_interfaces__srv__ColdStart_Request__copy(
  const ublox_ubx_interfaces__srv__ColdStart_Request * input,
  ublox_ubx_interfaces__srv__ColdStart_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // reset_type
  output->reset_type = input->reset_type;
  return true;
}

ublox_ubx_interfaces__srv__ColdStart_Request *
ublox_ubx_interfaces__srv__ColdStart_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_interfaces__srv__ColdStart_Request * msg = (ublox_ubx_interfaces__srv__ColdStart_Request *)allocator.allocate(sizeof(ublox_ubx_interfaces__srv__ColdStart_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_interfaces__srv__ColdStart_Request));
  bool success = ublox_ubx_interfaces__srv__ColdStart_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_interfaces__srv__ColdStart_Request__destroy(ublox_ubx_interfaces__srv__ColdStart_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_interfaces__srv__ColdStart_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_interfaces__srv__ColdStart_Request__Sequence__init(ublox_ubx_interfaces__srv__ColdStart_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_interfaces__srv__ColdStart_Request * data = NULL;

  if (size) {
    data = (ublox_ubx_interfaces__srv__ColdStart_Request *)allocator.zero_allocate(size, sizeof(ublox_ubx_interfaces__srv__ColdStart_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_interfaces__srv__ColdStart_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_interfaces__srv__ColdStart_Request__fini(&data[i - 1]);
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
ublox_ubx_interfaces__srv__ColdStart_Request__Sequence__fini(ublox_ubx_interfaces__srv__ColdStart_Request__Sequence * array)
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
      ublox_ubx_interfaces__srv__ColdStart_Request__fini(&array->data[i]);
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

ublox_ubx_interfaces__srv__ColdStart_Request__Sequence *
ublox_ubx_interfaces__srv__ColdStart_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_interfaces__srv__ColdStart_Request__Sequence * array = (ublox_ubx_interfaces__srv__ColdStart_Request__Sequence *)allocator.allocate(sizeof(ublox_ubx_interfaces__srv__ColdStart_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_interfaces__srv__ColdStart_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_interfaces__srv__ColdStart_Request__Sequence__destroy(ublox_ubx_interfaces__srv__ColdStart_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_interfaces__srv__ColdStart_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_interfaces__srv__ColdStart_Request__Sequence__are_equal(const ublox_ubx_interfaces__srv__ColdStart_Request__Sequence * lhs, const ublox_ubx_interfaces__srv__ColdStart_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_interfaces__srv__ColdStart_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_interfaces__srv__ColdStart_Request__Sequence__copy(
  const ublox_ubx_interfaces__srv__ColdStart_Request__Sequence * input,
  ublox_ubx_interfaces__srv__ColdStart_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_interfaces__srv__ColdStart_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_interfaces__srv__ColdStart_Request * data =
      (ublox_ubx_interfaces__srv__ColdStart_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_interfaces__srv__ColdStart_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_interfaces__srv__ColdStart_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_interfaces__srv__ColdStart_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ublox_ubx_interfaces__srv__ColdStart_Response__init(ublox_ubx_interfaces__srv__ColdStart_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
ublox_ubx_interfaces__srv__ColdStart_Response__fini(ublox_ubx_interfaces__srv__ColdStart_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
ublox_ubx_interfaces__srv__ColdStart_Response__are_equal(const ublox_ubx_interfaces__srv__ColdStart_Response * lhs, const ublox_ubx_interfaces__srv__ColdStart_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
ublox_ubx_interfaces__srv__ColdStart_Response__copy(
  const ublox_ubx_interfaces__srv__ColdStart_Response * input,
  ublox_ubx_interfaces__srv__ColdStart_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

ublox_ubx_interfaces__srv__ColdStart_Response *
ublox_ubx_interfaces__srv__ColdStart_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_interfaces__srv__ColdStart_Response * msg = (ublox_ubx_interfaces__srv__ColdStart_Response *)allocator.allocate(sizeof(ublox_ubx_interfaces__srv__ColdStart_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_interfaces__srv__ColdStart_Response));
  bool success = ublox_ubx_interfaces__srv__ColdStart_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_interfaces__srv__ColdStart_Response__destroy(ublox_ubx_interfaces__srv__ColdStart_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_interfaces__srv__ColdStart_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_interfaces__srv__ColdStart_Response__Sequence__init(ublox_ubx_interfaces__srv__ColdStart_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_interfaces__srv__ColdStart_Response * data = NULL;

  if (size) {
    data = (ublox_ubx_interfaces__srv__ColdStart_Response *)allocator.zero_allocate(size, sizeof(ublox_ubx_interfaces__srv__ColdStart_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_interfaces__srv__ColdStart_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_interfaces__srv__ColdStart_Response__fini(&data[i - 1]);
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
ublox_ubx_interfaces__srv__ColdStart_Response__Sequence__fini(ublox_ubx_interfaces__srv__ColdStart_Response__Sequence * array)
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
      ublox_ubx_interfaces__srv__ColdStart_Response__fini(&array->data[i]);
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

ublox_ubx_interfaces__srv__ColdStart_Response__Sequence *
ublox_ubx_interfaces__srv__ColdStart_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_interfaces__srv__ColdStart_Response__Sequence * array = (ublox_ubx_interfaces__srv__ColdStart_Response__Sequence *)allocator.allocate(sizeof(ublox_ubx_interfaces__srv__ColdStart_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_interfaces__srv__ColdStart_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_interfaces__srv__ColdStart_Response__Sequence__destroy(ublox_ubx_interfaces__srv__ColdStart_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_interfaces__srv__ColdStart_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_interfaces__srv__ColdStart_Response__Sequence__are_equal(const ublox_ubx_interfaces__srv__ColdStart_Response__Sequence * lhs, const ublox_ubx_interfaces__srv__ColdStart_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_interfaces__srv__ColdStart_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_interfaces__srv__ColdStart_Response__Sequence__copy(
  const ublox_ubx_interfaces__srv__ColdStart_Response__Sequence * input,
  ublox_ubx_interfaces__srv__ColdStart_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_interfaces__srv__ColdStart_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_interfaces__srv__ColdStart_Response * data =
      (ublox_ubx_interfaces__srv__ColdStart_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_interfaces__srv__ColdStart_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_interfaces__srv__ColdStart_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_interfaces__srv__ColdStart_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
