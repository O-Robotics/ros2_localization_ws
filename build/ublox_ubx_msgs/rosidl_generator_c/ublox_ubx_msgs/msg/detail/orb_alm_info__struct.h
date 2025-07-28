// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/OrbAlmInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_ALM_INFO__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_ALM_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ALM_USABILITY_UNKNOWN'.
/**
  * The usability period is unknown
 */
enum
{
  ublox_ubx_msgs__msg__OrbAlmInfo__ALM_USABILITY_UNKNOWN = 31
};

/// Constant 'ALM_USABILITY_OVER_30_DAYS'.
/**
  * The usability period is more than 30 days
 */
enum
{
  ublox_ubx_msgs__msg__OrbAlmInfo__ALM_USABILITY_OVER_30_DAYS = 30
};

/// Constant 'ALM_USABILITY_EXPIRED'.
/**
  * Almanac can no longer be used
 */
enum
{
  ublox_ubx_msgs__msg__OrbAlmInfo__ALM_USABILITY_EXPIRED = 0
};

/// Constant 'ALM_SOURCE_NOT_AVAILABLE'.
enum
{
  ublox_ubx_msgs__msg__OrbAlmInfo__ALM_SOURCE_NOT_AVAILABLE = 0
};

/// Constant 'ALM_SOURCE_GNSS_TRANSMISSION'.
enum
{
  ublox_ubx_msgs__msg__OrbAlmInfo__ALM_SOURCE_GNSS_TRANSMISSION = 1
};

/// Constant 'ALM_SOURCE_EXTERNAL_AIDING'.
enum
{
  ublox_ubx_msgs__msg__OrbAlmInfo__ALM_SOURCE_EXTERNAL_AIDING = 2
};

/// Struct defined in msg/OrbAlmInfo in the package ublox_ubx_msgs.
/**
  * Bit field structure for alm
 */
typedef struct ublox_ubx_msgs__msg__OrbAlmInfo
{
  /// How long the receiver will be able to use the stored almanac data from now on.
  /// For 30 > n > 0, the usability period is between n-1 and n days.
  uint8_t alm_usability;
  /// Almanac source
  uint8_t alm_source;
} ublox_ubx_msgs__msg__OrbAlmInfo;

// Struct for a sequence of ublox_ubx_msgs__msg__OrbAlmInfo.
typedef struct ublox_ubx_msgs__msg__OrbAlmInfo__Sequence
{
  ublox_ubx_msgs__msg__OrbAlmInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__OrbAlmInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_ALM_INFO__STRUCT_H_
