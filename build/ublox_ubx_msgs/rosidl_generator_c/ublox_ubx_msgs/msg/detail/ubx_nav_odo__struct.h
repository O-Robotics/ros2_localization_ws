// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavOdo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ODO__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ODO__STRUCT_H_

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

/// Struct defined in msg/UBXNavOdo in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-ODO (0x01 0x09) record Odometer solution
 */
typedef struct ublox_ubx_msgs__msg__UBXNavOdo
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// message version
  uint8_t version;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// m - ground distance since last reset
  uint32_t distance;
  /// m - total comulative ground distance (since last cold start)
  uint32_t total_distance;
  /// m - ground distance accuracy (1-sigma)
  uint32_t distance_std;
} ublox_ubx_msgs__msg__UBXNavOdo;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavOdo.
typedef struct ublox_ubx_msgs__msg__UBXNavOdo__Sequence
{
  ublox_ubx_msgs__msg__UBXNavOdo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavOdo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ODO__STRUCT_H_
