// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavSBAS.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODE_DISABLED'.
/**
  * Constants for SBAS Mode
 */
enum
{
  ublox_ubx_msgs__msg__UBXNavSBAS__MODE_DISABLED = 0
};

/// Constant 'MODE_ENABLED_INTEGRITY'.
enum
{
  ublox_ubx_msgs__msg__UBXNavSBAS__MODE_ENABLED_INTEGRITY = 1
};

/// Constant 'MODE_ENABLED_TEST'.
enum
{
  ublox_ubx_msgs__msg__UBXNavSBAS__MODE_ENABLED_TEST = 3
};

/// Constant 'SYS_UNKNOWN'.
/**
  * Constants for SBAS System
 */
enum
{
  ublox_ubx_msgs__msg__UBXNavSBAS__SYS_UNKNOWN = -1
};

/// Constant 'SYS_WAAS'.
enum
{
  ublox_ubx_msgs__msg__UBXNavSBAS__SYS_WAAS = 0
};

/// Constant 'SYS_EGNOS'.
enum
{
  ublox_ubx_msgs__msg__UBXNavSBAS__SYS_EGNOS = 1
};

/// Constant 'SYS_MSAS'.
enum
{
  ublox_ubx_msgs__msg__UBXNavSBAS__SYS_MSAS = 2
};

/// Constant 'SYS_GAGAN'.
enum
{
  ublox_ubx_msgs__msg__UBXNavSBAS__SYS_GAGAN = 3
};

/// Constant 'SYS_GPS'.
enum
{
  ublox_ubx_msgs__msg__UBXNavSBAS__SYS_GPS = 16
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'service'
#include "ublox_ubx_msgs/msg/detail/sbas_service__struct.h"
// Member 'status_flags'
#include "ublox_ubx_msgs/msg/detail/sbas_status_flags__struct.h"
// Member 'sv_data'
#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__struct.h"

/// Struct defined in msg/UBXNavSBAS in the package ublox_ubx_msgs.
/**
  * UBX-NAV-SBAS (0x01 0x32) - SBAS status data
 */
typedef struct ublox_ubx_msgs__msg__UBXNavSBAS
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// PRN Number of the GEO where correction and integrity data is used from
  uint8_t geo;
  /// SBAS Mode
  uint8_t mode;
  /// SBAS System (WAAS/EGNOS/...)
  int8_t sys;
  /// SBAS Services available
  ublox_ubx_msgs__msg__SBASService service;
  /// Number of SV data following
  uint8_t cnt;
  /// SBAS status flags
  ublox_ubx_msgs__msg__SBASStatusFlags status_flags;
  /// Reserved
  uint8_t reserved_0[2];
  /// Repeated group of data for each satellite
  ublox_ubx_msgs__msg__SBASSvData__Sequence sv_data;
} ublox_ubx_msgs__msg__UBXNavSBAS;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavSBAS.
typedef struct ublox_ubx_msgs__msg__UBXNavSBAS__Sequence
{
  ublox_ubx_msgs__msg__UBXNavSBAS * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavSBAS__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__STRUCT_H_
