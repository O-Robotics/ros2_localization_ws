// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavTimeUTC.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__STRUCT_H_

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
// Member 'utc_std'
#include "ublox_ubx_msgs/msg/detail/utc_std__struct.h"

/// Struct defined in msg/UBXNavTimeUTC in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-EOE (0x01 0x21) record UTC Time Solution
 */
typedef struct ublox_ubx_msgs__msg__UBXNavTimeUTC
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// ns - time accuracy estimate (utc)
  uint32_t t_acc;
  /// ns - fraction of second, range -1e9 to 1e9 (utc)
  int32_t nano;
  /// y - year utc
  int16_t year;
  /// month - month utc
  int8_t month;
  /// d -day utc
  int8_t day;
  /// h - hour utc
  int8_t hour;
  /// min - min utc
  int8_t min;
  /// s - sec utc
  int8_t sec;
  /// valid Time of Week
  bool valid_tow;
  /// valid Week Number
  bool valid_wkn;
  /// valid UTC time
  bool valid_utc;
  /// UTC standard identifier
  ublox_ubx_msgs__msg__UtcStd utc_std;
} ublox_ubx_msgs__msg__UBXNavTimeUTC;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavTimeUTC.
typedef struct ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence
{
  ublox_ubx_msgs__msg__UBXNavTimeUTC * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavTimeUTC__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__STRUCT_H_
