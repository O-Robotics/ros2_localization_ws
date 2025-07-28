// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavOrb.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__STRUCT_H_

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
// Member 'sv_info'
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__struct.h"

/// Struct defined in msg/UBXNavOrb in the package ublox_ubx_msgs.
/**
  * UBX-NAV-ORB (0x01 0x34) - GNSS orbit database info
 */
typedef struct ublox_ubx_msgs__msg__UBXNavOrb
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// Message version (0x01 for this version)
  uint8_t version;
  /// Number of SVs in the database
  uint8_t num_sv;
  /// Reserved
  uint8_t reserved_0[2];
  /// Repeated group of data for each SV
  ublox_ubx_msgs__msg__OrbSVInfo__Sequence sv_info;
} ublox_ubx_msgs__msg__UBXNavOrb;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavOrb.
typedef struct ublox_ubx_msgs__msg__UBXNavOrb__Sequence
{
  ublox_ubx_msgs__msg__UBXNavOrb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavOrb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__STRUCT_H_
