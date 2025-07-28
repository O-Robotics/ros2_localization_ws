// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/SatInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'flags'
#include "ublox_ubx_msgs/msg/detail/sat_flags__struct.h"

/// Struct defined in msg/SatInfo in the package ublox_ubx_msgs.
/**
  * Information for each Satellite Vehicle
 */
typedef struct ublox_ubx_msgs__msg__SatInfo
{
  /// GNSS identifier
  uint8_t gnss_id;
  /// Satellite identifier
  uint8_t sv_id;
  /// dBHz - Carrier to noise ratio (signal strength)
  uint8_t cno;
  /// deg - Elevation (range: +/-90), unknown if out of range
  int8_t elev;
  /// deg - Azimuth (range 0-360), unknown if elevation is out of range
  int16_t azim;
  /// Pseudorange residual (scaled by 0.1)
  int16_t pr_res;
  /// Bitmask with satellite information
  ublox_ubx_msgs__msg__SatFlags flags;
} ublox_ubx_msgs__msg__SatInfo;

// Struct for a sequence of ublox_ubx_msgs__msg__SatInfo.
typedef struct ublox_ubx_msgs__msg__SatInfo__Sequence
{
  ublox_ubx_msgs__msg__SatInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__SatInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__STRUCT_H_
