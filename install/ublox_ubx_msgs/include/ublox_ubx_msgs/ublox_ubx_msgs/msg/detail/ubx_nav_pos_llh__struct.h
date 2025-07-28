// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavPosLLH.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_LLH__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_LLH__STRUCT_H_

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

/// Struct defined in msg/UBXNavPosLLH in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-POSLLH (0x01 0x02) record 
 */
typedef struct ublox_ubx_msgs__msg__UBXNavPosLLH
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// deg scale 1e-7 - longitude
  int32_t lon;
  /// deg scale 1e-7 - latitude
  int32_t lat;
  /// mm - height above ellipsoid
  int32_t height;
  /// mm - height above mean sea level
  int32_t hmsl;
  /// mm - horizontal accuracy estimate
  uint32_t h_acc;
  /// mm - vertical accuracy estimate
  uint32_t v_acc;
} ublox_ubx_msgs__msg__UBXNavPosLLH;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavPosLLH.
typedef struct ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence
{
  ublox_ubx_msgs__msg__UBXNavPosLLH * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavPosLLH__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_LLH__STRUCT_H_
