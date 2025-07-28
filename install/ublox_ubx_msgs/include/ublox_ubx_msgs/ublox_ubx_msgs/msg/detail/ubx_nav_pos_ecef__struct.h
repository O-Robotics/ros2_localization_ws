// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavPosECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__STRUCT_H_

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

/// Struct defined in msg/UBXNavPosECEF in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-POSECEF (0x01 0x01) record 
 */
typedef struct ublox_ubx_msgs__msg__UBXNavPosECEF
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// cm - ECEF X coordinate
  int32_t ecef_x;
  /// cm - ECEF Y coordinate
  int32_t ecef_y;
  /// cm - ECEF Z coordinate
  int32_t ecef_z;
  /// cm - position accuracy estimate
  uint32_t p_acc;
} ublox_ubx_msgs__msg__UBXNavPosECEF;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavPosECEF.
typedef struct ublox_ubx_msgs__msg__UBXNavPosECEF__Sequence
{
  ublox_ubx_msgs__msg__UBXNavPosECEF * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavPosECEF__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__STRUCT_H_
