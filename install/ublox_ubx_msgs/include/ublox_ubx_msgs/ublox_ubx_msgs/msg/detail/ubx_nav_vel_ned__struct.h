// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavVelNED.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__STRUCT_H_

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

/// Struct defined in msg/UBXNavVelNED in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-VELNED (0x01 0x12) record
  * Velocity solution in NED frame
 */
typedef struct ublox_ubx_msgs__msg__UBXNavVelNED
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// cm/s - NED north velocity
  int32_t vel_n;
  /// cm/s - NED east velocity
  int32_t vel_e;
  /// cm/s - NED down velocity
  int32_t vel_d;
  /// cm/s - speed (3-D)
  uint32_t speed;
  /// cm/s - ground speed (2-D)
  uint32_t g_speed;
  /// deg scale ie-5 - heading of motion (2-D)
  int32_t heading;
  /// cm/s - speed accuracy estimate
  uint32_t s_acc;
  /// deg scale ie-5 - course/heading accruracy estime (both motion and vehicle)
  uint32_t c_acc;
} ublox_ubx_msgs__msg__UBXNavVelNED;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavVelNED.
typedef struct ublox_ubx_msgs__msg__UBXNavVelNED__Sequence
{
  ublox_ubx_msgs__msg__UBXNavVelNED * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavVelNED__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__STRUCT_H_
