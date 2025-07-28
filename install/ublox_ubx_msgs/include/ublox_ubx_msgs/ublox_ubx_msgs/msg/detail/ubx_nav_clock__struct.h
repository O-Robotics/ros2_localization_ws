// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavClock.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/UBXNavClock in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-CLOCK (0x01 0x22) record clock solution
 */
typedef struct ublox_ubx_msgs__msg__UBXNavClock
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// ns - Clock bias
  int32_t clk_b;
  /// ns/s - Clock drift
  int32_t clk_d;
  /// ns - Time accuracy estimate
  uint32_t t_acc;
  /// ps/s - Frequency accuracy estimate
  uint32_t f_acc;
} ublox_ubx_msgs__msg__UBXNavClock;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavClock.
typedef struct ublox_ubx_msgs__msg__UBXNavClock__Sequence
{
  ublox_ubx_msgs__msg__UBXNavClock * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavClock__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__STRUCT_H_
