// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/OrbSVInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'sv_flag'
#include "ublox_ubx_msgs/msg/detail/orb_sv_flag__struct.h"
// Member 'eph'
#include "ublox_ubx_msgs/msg/detail/orb_eph_info__struct.h"
// Member 'alm'
#include "ublox_ubx_msgs/msg/detail/orb_alm_info__struct.h"
// Member 'other_orb'
#include "ublox_ubx_msgs/msg/detail/other_orb_info__struct.h"

/// Struct defined in msg/OrbSVInfo in the package ublox_ubx_msgs.
/**
  * Information for each Satellite Vehicle (SV) in the GNSS orbit database
 */
typedef struct ublox_ubx_msgs__msg__OrbSVInfo
{
  /// GNSS ID
  uint8_t gnss_id;
  /// Satellite ID
  uint8_t sv_id;
  /// Information Flags for SV
  ublox_ubx_msgs__msg__OrbSVFlag sv_flag;
  /// Ephemeris data
  ublox_ubx_msgs__msg__OrbEphInfo eph;
  /// Almanac data
  ublox_ubx_msgs__msg__OrbAlmInfo alm;
  /// Other orbit data available
  ublox_ubx_msgs__msg__OtherOrbInfo other_orb;
} ublox_ubx_msgs__msg__OrbSVInfo;

// Struct for a sequence of ublox_ubx_msgs__msg__OrbSVInfo.
typedef struct ublox_ubx_msgs__msg__OrbSVInfo__Sequence
{
  ublox_ubx_msgs__msg__OrbSVInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__OrbSVInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__STRUCT_H_
