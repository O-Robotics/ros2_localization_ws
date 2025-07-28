// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/OrbSVFlag.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'HEALTH_UNKNOWN'.
enum
{
  ublox_ubx_msgs__msg__OrbSVFlag__HEALTH_UNKNOWN = 0
};

/// Constant 'HEALTH_HEALTHY'.
enum
{
  ublox_ubx_msgs__msg__OrbSVFlag__HEALTH_HEALTHY = 1
};

/// Constant 'HEALTH_NOT_HEALTHY'.
enum
{
  ublox_ubx_msgs__msg__OrbSVFlag__HEALTH_NOT_HEALTHY = 2
};

/// Constant 'VISIBILITY_UNKNOWN'.
enum
{
  ublox_ubx_msgs__msg__OrbSVFlag__VISIBILITY_UNKNOWN = 0
};

/// Constant 'VISIBILITY_BELOW_HORIZON'.
enum
{
  ublox_ubx_msgs__msg__OrbSVFlag__VISIBILITY_BELOW_HORIZON = 1
};

/// Constant 'VISIBILITY_ABOVE_HORIZON'.
enum
{
  ublox_ubx_msgs__msg__OrbSVFlag__VISIBILITY_ABOVE_HORIZON = 2
};

/// Constant 'VISIBILITY_ABOVE_ELEVATION_MASK'.
enum
{
  ublox_ubx_msgs__msg__OrbSVFlag__VISIBILITY_ABOVE_ELEVATION_MASK = 3
};

/// Struct defined in msg/OrbSVFlag in the package ublox_ubx_msgs.
/**
  * Bit field structure for svFlag
 */
typedef struct ublox_ubx_msgs__msg__OrbSVFlag
{
  /// SV health
  uint8_t health;
  /// SV visibility
  uint8_t visibility;
} ublox_ubx_msgs__msg__OrbSVFlag;

// Struct for a sequence of ublox_ubx_msgs__msg__OrbSVFlag.
typedef struct ublox_ubx_msgs__msg__OrbSVFlag__Sequence
{
  ublox_ubx_msgs__msg__OrbSVFlag * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__OrbSVFlag__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__STRUCT_H_
