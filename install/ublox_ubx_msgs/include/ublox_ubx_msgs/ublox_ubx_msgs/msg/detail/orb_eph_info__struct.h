// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/OrbEphInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'EPH_USABILITY_UNKNOWN'.
/**
  * The usability period is unknown
 */
enum
{
  ublox_ubx_msgs__msg__OrbEphInfo__EPH_USABILITY_UNKNOWN = 31
};

/// Constant 'EPH_USABILITY_OVER_450_MIN'.
/**
  * The usability period is more than 450 minutes
 */
enum
{
  ublox_ubx_msgs__msg__OrbEphInfo__EPH_USABILITY_OVER_450_MIN = 30
};

/// Constant 'EPH_USABILITY_EXPIRED'.
/**
  * Ephemeris can no longer be used
 */
enum
{
  ublox_ubx_msgs__msg__OrbEphInfo__EPH_USABILITY_EXPIRED = 0
};

/// Constant 'EPH_SOURCE_NOT_AVAILABLE'.
enum
{
  ublox_ubx_msgs__msg__OrbEphInfo__EPH_SOURCE_NOT_AVAILABLE = 0
};

/// Constant 'EPH_SOURCE_GNSS_TRANSMISSION'.
enum
{
  ublox_ubx_msgs__msg__OrbEphInfo__EPH_SOURCE_GNSS_TRANSMISSION = 1
};

/// Constant 'EPH_SOURCE_EXTERNAL_AIDING'.
enum
{
  ublox_ubx_msgs__msg__OrbEphInfo__EPH_SOURCE_EXTERNAL_AIDING = 2
};

/// Struct defined in msg/OrbEphInfo in the package ublox_ubx_msgs.
/**
  * Bit field structure for eph
 */
typedef struct ublox_ubx_msgs__msg__OrbEphInfo
{
  /// How long the receiver will be able to use the stored ephemeris data from now on.
  /// For 30 > n > 0, the usability period is between (n-1)*15 and n*15 minutes.
  uint8_t eph_usability;
  /// Ephemeris source
  uint8_t eph_source;
} ublox_ubx_msgs__msg__OrbEphInfo;

// Struct for a sequence of ublox_ubx_msgs__msg__OrbEphInfo.
typedef struct ublox_ubx_msgs__msg__OrbEphInfo__Sequence
{
  ublox_ubx_msgs__msg__OrbEphInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__OrbEphInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__STRUCT_H_
