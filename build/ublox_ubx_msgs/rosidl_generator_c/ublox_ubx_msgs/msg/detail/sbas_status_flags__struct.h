// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/SBASStatusFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'INTEGRITY_UNKNOWN'.
enum
{
  ublox_ubx_msgs__msg__SBASStatusFlags__INTEGRITY_UNKNOWN = 0
};

/// Constant 'INTEGRITY_NOT_AVAILABLE'.
enum
{
  ublox_ubx_msgs__msg__SBASStatusFlags__INTEGRITY_NOT_AVAILABLE = 1
};

/// Constant 'INTEGRITY_USED'.
enum
{
  ublox_ubx_msgs__msg__SBASStatusFlags__INTEGRITY_USED = 2
};

/// Struct defined in msg/SBASStatusFlags in the package ublox_ubx_msgs.
/**
  * SBAS status flags
 */
typedef struct ublox_ubx_msgs__msg__SBASStatusFlags
{
  /// SBAS integrity used
  uint8_t integrity_used;
} ublox_ubx_msgs__msg__SBASStatusFlags;

// Struct for a sequence of ublox_ubx_msgs__msg__SBASStatusFlags.
typedef struct ublox_ubx_msgs__msg__SBASStatusFlags__Sequence
{
  ublox_ubx_msgs__msg__SBASStatusFlags * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__SBASStatusFlags__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__STRUCT_H_
