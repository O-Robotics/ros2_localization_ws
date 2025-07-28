// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/SigFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__STRUCT_H_

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
  ublox_ubx_msgs__msg__SigFlags__HEALTH_UNKNOWN = 0
};

/// Constant 'HEALTH_HEALTHY'.
enum
{
  ublox_ubx_msgs__msg__SigFlags__HEALTH_HEALTHY = 1
};

/// Constant 'HEALTH_UNHEALTHY'.
enum
{
  ublox_ubx_msgs__msg__SigFlags__HEALTH_UNHEALTHY = 2
};

/// Struct defined in msg/SigFlags in the package ublox_ubx_msgs.
/**
  * Signal related flags
 */
typedef struct ublox_ubx_msgs__msg__SigFlags
{
  /// Signal health flag
  uint8_t health;
  /// Pseudorange has been smoothed
  bool pr_smoothed;
  /// Pseudorange has been used for this signal
  bool pr_used;
  /// Carrier range has been used for this signal
  bool cr_used;
  /// Range rate (Doppler) has been used for this signal
  bool do_used;
  /// Pseudorange corrections have been used for this signal
  bool pr_corr_used;
  /// Carrier range corrections have been used for this signal
  bool cr_corr_used;
  /// Range rate (Doppler) corrections have been used for this signal
  bool do_corr_used;
} ublox_ubx_msgs__msg__SigFlags;

// Struct for a sequence of ublox_ubx_msgs__msg__SigFlags.
typedef struct ublox_ubx_msgs__msg__SigFlags__Sequence
{
  ublox_ubx_msgs__msg__SigFlags * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__SigFlags__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__STRUCT_H_
