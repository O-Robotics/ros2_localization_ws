// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavVelECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__STRUCT_H_

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

/// Struct defined in msg/UBXNavVelECEF in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-VELECEF (0x01 0x11) record
  * Velocity solution in ECEF
 */
typedef struct ublox_ubx_msgs__msg__UBXNavVelECEF
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// cm/s - ECEF X velocity
  int32_t ecef_vx;
  /// cm/s - ECEF Y velocity
  int32_t ecef_vy;
  /// cm/s - ECEF Z velocity
  int32_t ecef_vz;
  /// cm/s - velocity accuracy estimate
  uint32_t s_acc;
} ublox_ubx_msgs__msg__UBXNavVelECEF;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavVelECEF.
typedef struct ublox_ubx_msgs__msg__UBXNavVelECEF__Sequence
{
  ublox_ubx_msgs__msg__UBXNavVelECEF * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavVelECEF__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__STRUCT_H_
