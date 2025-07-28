// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/SatFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'QUALITY_NO_SIGNAL'.
/**
  * Signal quality indicator
 */
enum
{
  ublox_ubx_msgs__msg__SatFlags__QUALITY_NO_SIGNAL = 0
};

/// Constant 'QUALITY_SEARCHING'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__QUALITY_SEARCHING = 1
};

/// Constant 'QUALITY_ACQUIRED'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__QUALITY_ACQUIRED = 2
};

/// Constant 'QUALITY_UNUSABLE'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__QUALITY_UNUSABLE = 3
};

/// Constant 'QUALITY_CODE_LOCKED'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__QUALITY_CODE_LOCKED = 4
};

/// Constant 'QUALITY_CARRIER_LOCKED'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__QUALITY_CARRIER_LOCKED = 5
};

/// Constant 'HEALTH_UNKNOWN'.
/**
  * Signal health flag
 */
enum
{
  ublox_ubx_msgs__msg__SatFlags__HEALTH_UNKNOWN = 0
};

/// Constant 'HEALTH_HEALTHY'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__HEALTH_HEALTHY = 1
};

/// Constant 'HEALTH_UNHEALTHY'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__HEALTH_UNHEALTHY = 2
};

/// Constant 'ORBIT_NO_INFO'.
/**
  * Orbit source
 */
enum
{
  ublox_ubx_msgs__msg__SatFlags__ORBIT_NO_INFO = 0
};

/// Constant 'ORBIT_EPH_USED'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__ORBIT_EPH_USED = 1
};

/// Constant 'ORBIT_ALM_USED'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__ORBIT_ALM_USED = 2
};

/// Constant 'ORBIT_ASSISTNOW_OFFLINE'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__ORBIT_ASSISTNOW_OFFLINE = 3
};

/// Constant 'ORBIT_ASSISTNOW_AUTONOMOUS'.
enum
{
  ublox_ubx_msgs__msg__SatFlags__ORBIT_ASSISTNOW_AUTONOMOUS = 4
};

/// Struct defined in msg/SatFlags in the package ublox_ubx_msgs.
/**
  * Bit field structure for flags in UBX-NAV-SAT (0x01 0x35)
 */
typedef struct ublox_ubx_msgs__msg__SatFlags
{
  uint8_t quality_ind;
  /// Signal currently being used for navigation
  bool sv_used;
  uint8_t health;
  /// Differential correction data availability
  bool diff_corr;
  /// Carrier smoothed pseudorange used
  bool smoothed;
  uint8_t orbit_source;
  /// Ephemeris availability
  bool eph_avail;
  /// Almanac availability
  bool alm_avail;
  /// AssistNow Offline data availability
  bool ano_avail;
  /// AssistNow Autonomous data availability
  bool aop_avail;
  /// Corrections used flags
  bool sbas_corr_used;
  bool rtcm_corr_used;
  bool slas_corr_used;
  bool spartn_corr_used;
  bool pr_corr_used;
  bool cr_corr_used;
  bool do_corr_used;
  bool clas_corr_used;
} ublox_ubx_msgs__msg__SatFlags;

// Struct for a sequence of ublox_ubx_msgs__msg__SatFlags.
typedef struct ublox_ubx_msgs__msg__SatFlags__Sequence
{
  ublox_ubx_msgs__msg__SatFlags * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__SatFlags__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__STRUCT_H_
