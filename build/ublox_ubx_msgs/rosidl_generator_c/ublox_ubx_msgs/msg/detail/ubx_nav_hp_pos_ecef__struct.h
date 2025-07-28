// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__STRUCT_H_

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

/// Struct defined in msg/UBXNavHPPosECEF in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-HPPOSECEF (0x01 0x13) record 
 */
typedef struct ublox_ubx_msgs__msg__UBXNavHPPosECEF
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// message version
  uint8_t version;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// cm - ECEF X coordinate
  int32_t ecef_x;
  /// cm - ECEF Y coordinate
  int32_t ecef_y;
  /// cm - ECEF Z coordinate
  int32_t ecef_z;
  /// HPs must be in the range of -99..+99. Precise coordinate in cm = ecef_? + (ecef_?_hp * 1e-2)
  /// mm scale 0.1 - high precision ECEF X coordinate
  int8_t ecef_x_hp;
  /// mm scale 0.1 - high precision ECEF Y coordinate
  int8_t ecef_y_hp;
  /// mm scale 0.1 - high precision ECEF Z coordinate
  int8_t ecef_z_hp;
  bool invalid_ecef_x;
  bool invalid_ecef_y;
  bool invalid_ecef_z;
  bool invalid_ecef_x_hp;
  bool invalid_ecef_y_hp;
  bool invalid_ecef_z_hp;
  /// mm scale 0.1 - position accuracy estimate
  uint32_t p_acc;
} ublox_ubx_msgs__msg__UBXNavHPPosECEF;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavHPPosECEF.
typedef struct ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence
{
  ublox_ubx_msgs__msg__UBXNavHPPosECEF * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavHPPosECEF__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__STRUCT_H_
