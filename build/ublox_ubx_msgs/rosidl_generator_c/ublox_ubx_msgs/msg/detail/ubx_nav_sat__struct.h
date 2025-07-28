// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavSat.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SAT__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SAT__STRUCT_H_

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
#include "ublox_ubx_msgs/msg/detail/sat_info__struct.h"

/// Struct defined in msg/UBXNavSat in the package ublox_ubx_msgs.
/**
  * UBX-NAV-SAT (0x01 0x35) - Satellite information
 */
typedef struct ublox_ubx_msgs__msg__UBXNavSat
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// Message version (0x01 for this version)
  uint8_t version;
  /// Number of satellites
  uint8_t num_svs;
  /// uint8 reserved_0     # Reserved
  /// Repeated group of data for each satellite
  ublox_ubx_msgs__msg__SatInfo__Sequence sv_info;
} ublox_ubx_msgs__msg__UBXNavSat;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavSat.
typedef struct ublox_ubx_msgs__msg__UBXNavSat__Sequence
{
  ublox_ubx_msgs__msg__UBXNavSat * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavSat__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SAT__STRUCT_H_
