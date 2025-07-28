// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/SBASService.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SBASService in the package ublox_ubx_msgs.
/**
  * SBAS Services available
 */
typedef struct ublox_ubx_msgs__msg__SBASService
{
  /// GEO may be used as ranging source
  bool ranging;
  /// GEO is providing correction data
  bool corrections;
  /// GEO is providing integrity
  bool integrity;
  /// GEO is in test mode
  bool test_mode;
  /// Problem with signal or broadcast data indicated
  bool bad;
} ublox_ubx_msgs__msg__SBASService;

// Struct for a sequence of ublox_ubx_msgs__msg__SBASService.
typedef struct ublox_ubx_msgs__msg__SBASService__Sequence
{
  ublox_ubx_msgs__msg__SBASService * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__SBASService__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__STRUCT_H_
