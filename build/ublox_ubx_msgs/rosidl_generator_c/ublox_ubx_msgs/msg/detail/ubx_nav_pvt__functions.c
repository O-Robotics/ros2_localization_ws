// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ublox_ubx_msgs:msg/UBXNavPVT.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__functions.h"

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
// Member `psm`
#include "ublox_ubx_msgs/msg/detail/psmpvt__functions.h"
// Member `carr_soln`
#include "ublox_ubx_msgs/msg/detail/carr_soln__functions.h"

bool
ublox_ubx_msgs__msg__UBXNavPVT__init(ublox_ubx_msgs__msg__UBXNavPVT * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ublox_ubx_msgs__msg__UBXNavPVT__fini(msg);
    return false;
  }
  // itow
  // year
  // month
  // day
  // hour
  // min
  // sec
  // valid_date
  // valid_time
  // fully_resolved
  // valid_mag
  // t_acc
  // nano
  // gps_fix
  if (!ublox_ubx_msgs__msg__GpsFix__init(&msg->gps_fix)) {
    ublox_ubx_msgs__msg__UBXNavPVT__fini(msg);
    return false;
  }
  // gnss_fix_ok
  // diff_soln
  // psm
  if (!ublox_ubx_msgs__msg__PSMPVT__init(&msg->psm)) {
    ublox_ubx_msgs__msg__UBXNavPVT__fini(msg);
    return false;
  }
  // head_veh_valid
  // carr_soln
  if (!ublox_ubx_msgs__msg__CarrSoln__init(&msg->carr_soln)) {
    ublox_ubx_msgs__msg__UBXNavPVT__fini(msg);
    return false;
  }
  // confirmed_avail
  // confirmed_date
  // confirmed_time
  // num_sv
  // lon
  // lat
  // height
  // hmsl
  // h_acc
  // v_acc
  // vel_n
  // vel_e
  // vel_d
  // g_speed
  // head_mot
  // s_acc
  // head_acc
  // p_dop
  // invalid_llh
  // head_veh
  // mag_dec
  // mag_acc
  return true;
}

void
ublox_ubx_msgs__msg__UBXNavPVT__fini(ublox_ubx_msgs__msg__UBXNavPVT * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // itow
  // year
  // month
  // day
  // hour
  // min
  // sec
  // valid_date
  // valid_time
  // fully_resolved
  // valid_mag
  // t_acc
  // nano
  // gps_fix
  ublox_ubx_msgs__msg__GpsFix__fini(&msg->gps_fix);
  // gnss_fix_ok
  // diff_soln
  // psm
  ublox_ubx_msgs__msg__PSMPVT__fini(&msg->psm);
  // head_veh_valid
  // carr_soln
  ublox_ubx_msgs__msg__CarrSoln__fini(&msg->carr_soln);
  // confirmed_avail
  // confirmed_date
  // confirmed_time
  // num_sv
  // lon
  // lat
  // height
  // hmsl
  // h_acc
  // v_acc
  // vel_n
  // vel_e
  // vel_d
  // g_speed
  // head_mot
  // s_acc
  // head_acc
  // p_dop
  // invalid_llh
  // head_veh
  // mag_dec
  // mag_acc
}

bool
ublox_ubx_msgs__msg__UBXNavPVT__are_equal(const ublox_ubx_msgs__msg__UBXNavPVT * lhs, const ublox_ubx_msgs__msg__UBXNavPVT * rhs)
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
  // valid_date
  if (lhs->valid_date != rhs->valid_date) {
    return false;
  }
  // valid_time
  if (lhs->valid_time != rhs->valid_time) {
    return false;
  }
  // fully_resolved
  if (lhs->fully_resolved != rhs->fully_resolved) {
    return false;
  }
  // valid_mag
  if (lhs->valid_mag != rhs->valid_mag) {
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
  // gps_fix
  if (!ublox_ubx_msgs__msg__GpsFix__are_equal(
      &(lhs->gps_fix), &(rhs->gps_fix)))
  {
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
  // psm
  if (!ublox_ubx_msgs__msg__PSMPVT__are_equal(
      &(lhs->psm), &(rhs->psm)))
  {
    return false;
  }
  // head_veh_valid
  if (lhs->head_veh_valid != rhs->head_veh_valid) {
    return false;
  }
  // carr_soln
  if (!ublox_ubx_msgs__msg__CarrSoln__are_equal(
      &(lhs->carr_soln), &(rhs->carr_soln)))
  {
    return false;
  }
  // confirmed_avail
  if (lhs->confirmed_avail != rhs->confirmed_avail) {
    return false;
  }
  // confirmed_date
  if (lhs->confirmed_date != rhs->confirmed_date) {
    return false;
  }
  // confirmed_time
  if (lhs->confirmed_time != rhs->confirmed_time) {
    return false;
  }
  // num_sv
  if (lhs->num_sv != rhs->num_sv) {
    return false;
  }
  // lon
  if (lhs->lon != rhs->lon) {
    return false;
  }
  // lat
  if (lhs->lat != rhs->lat) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // hmsl
  if (lhs->hmsl != rhs->hmsl) {
    return false;
  }
  // h_acc
  if (lhs->h_acc != rhs->h_acc) {
    return false;
  }
  // v_acc
  if (lhs->v_acc != rhs->v_acc) {
    return false;
  }
  // vel_n
  if (lhs->vel_n != rhs->vel_n) {
    return false;
  }
  // vel_e
  if (lhs->vel_e != rhs->vel_e) {
    return false;
  }
  // vel_d
  if (lhs->vel_d != rhs->vel_d) {
    return false;
  }
  // g_speed
  if (lhs->g_speed != rhs->g_speed) {
    return false;
  }
  // head_mot
  if (lhs->head_mot != rhs->head_mot) {
    return false;
  }
  // s_acc
  if (lhs->s_acc != rhs->s_acc) {
    return false;
  }
  // head_acc
  if (lhs->head_acc != rhs->head_acc) {
    return false;
  }
  // p_dop
  if (lhs->p_dop != rhs->p_dop) {
    return false;
  }
  // invalid_llh
  if (lhs->invalid_llh != rhs->invalid_llh) {
    return false;
  }
  // head_veh
  if (lhs->head_veh != rhs->head_veh) {
    return false;
  }
  // mag_dec
  if (lhs->mag_dec != rhs->mag_dec) {
    return false;
  }
  // mag_acc
  if (lhs->mag_acc != rhs->mag_acc) {
    return false;
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavPVT__copy(
  const ublox_ubx_msgs__msg__UBXNavPVT * input,
  ublox_ubx_msgs__msg__UBXNavPVT * output)
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
  // valid_date
  output->valid_date = input->valid_date;
  // valid_time
  output->valid_time = input->valid_time;
  // fully_resolved
  output->fully_resolved = input->fully_resolved;
  // valid_mag
  output->valid_mag = input->valid_mag;
  // t_acc
  output->t_acc = input->t_acc;
  // nano
  output->nano = input->nano;
  // gps_fix
  if (!ublox_ubx_msgs__msg__GpsFix__copy(
      &(input->gps_fix), &(output->gps_fix)))
  {
    return false;
  }
  // gnss_fix_ok
  output->gnss_fix_ok = input->gnss_fix_ok;
  // diff_soln
  output->diff_soln = input->diff_soln;
  // psm
  if (!ublox_ubx_msgs__msg__PSMPVT__copy(
      &(input->psm), &(output->psm)))
  {
    return false;
  }
  // head_veh_valid
  output->head_veh_valid = input->head_veh_valid;
  // carr_soln
  if (!ublox_ubx_msgs__msg__CarrSoln__copy(
      &(input->carr_soln), &(output->carr_soln)))
  {
    return false;
  }
  // confirmed_avail
  output->confirmed_avail = input->confirmed_avail;
  // confirmed_date
  output->confirmed_date = input->confirmed_date;
  // confirmed_time
  output->confirmed_time = input->confirmed_time;
  // num_sv
  output->num_sv = input->num_sv;
  // lon
  output->lon = input->lon;
  // lat
  output->lat = input->lat;
  // height
  output->height = input->height;
  // hmsl
  output->hmsl = input->hmsl;
  // h_acc
  output->h_acc = input->h_acc;
  // v_acc
  output->v_acc = input->v_acc;
  // vel_n
  output->vel_n = input->vel_n;
  // vel_e
  output->vel_e = input->vel_e;
  // vel_d
  output->vel_d = input->vel_d;
  // g_speed
  output->g_speed = input->g_speed;
  // head_mot
  output->head_mot = input->head_mot;
  // s_acc
  output->s_acc = input->s_acc;
  // head_acc
  output->head_acc = input->head_acc;
  // p_dop
  output->p_dop = input->p_dop;
  // invalid_llh
  output->invalid_llh = input->invalid_llh;
  // head_veh
  output->head_veh = input->head_veh;
  // mag_dec
  output->mag_dec = input->mag_dec;
  // mag_acc
  output->mag_acc = input->mag_acc;
  return true;
}

ublox_ubx_msgs__msg__UBXNavPVT *
ublox_ubx_msgs__msg__UBXNavPVT__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavPVT * msg = (ublox_ubx_msgs__msg__UBXNavPVT *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavPVT), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ublox_ubx_msgs__msg__UBXNavPVT));
  bool success = ublox_ubx_msgs__msg__UBXNavPVT__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ublox_ubx_msgs__msg__UBXNavPVT__destroy(ublox_ubx_msgs__msg__UBXNavPVT * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ublox_ubx_msgs__msg__UBXNavPVT__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ublox_ubx_msgs__msg__UBXNavPVT__Sequence__init(ublox_ubx_msgs__msg__UBXNavPVT__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavPVT * data = NULL;

  if (size) {
    data = (ublox_ubx_msgs__msg__UBXNavPVT *)allocator.zero_allocate(size, sizeof(ublox_ubx_msgs__msg__UBXNavPVT), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ublox_ubx_msgs__msg__UBXNavPVT__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ublox_ubx_msgs__msg__UBXNavPVT__fini(&data[i - 1]);
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
ublox_ubx_msgs__msg__UBXNavPVT__Sequence__fini(ublox_ubx_msgs__msg__UBXNavPVT__Sequence * array)
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
      ublox_ubx_msgs__msg__UBXNavPVT__fini(&array->data[i]);
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

ublox_ubx_msgs__msg__UBXNavPVT__Sequence *
ublox_ubx_msgs__msg__UBXNavPVT__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ublox_ubx_msgs__msg__UBXNavPVT__Sequence * array = (ublox_ubx_msgs__msg__UBXNavPVT__Sequence *)allocator.allocate(sizeof(ublox_ubx_msgs__msg__UBXNavPVT__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ublox_ubx_msgs__msg__UBXNavPVT__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ublox_ubx_msgs__msg__UBXNavPVT__Sequence__destroy(ublox_ubx_msgs__msg__UBXNavPVT__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ublox_ubx_msgs__msg__UBXNavPVT__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ublox_ubx_msgs__msg__UBXNavPVT__Sequence__are_equal(const ublox_ubx_msgs__msg__UBXNavPVT__Sequence * lhs, const ublox_ubx_msgs__msg__UBXNavPVT__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavPVT__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ublox_ubx_msgs__msg__UBXNavPVT__Sequence__copy(
  const ublox_ubx_msgs__msg__UBXNavPVT__Sequence * input,
  ublox_ubx_msgs__msg__UBXNavPVT__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ublox_ubx_msgs__msg__UBXNavPVT);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ublox_ubx_msgs__msg__UBXNavPVT * data =
      (ublox_ubx_msgs__msg__UBXNavPVT *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ublox_ubx_msgs__msg__UBXNavPVT__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ublox_ubx_msgs__msg__UBXNavPVT__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ublox_ubx_msgs__msg__UBXNavPVT__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
