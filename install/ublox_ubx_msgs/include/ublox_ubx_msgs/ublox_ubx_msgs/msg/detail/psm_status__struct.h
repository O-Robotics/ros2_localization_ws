// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/PSMStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__PSM_STATUS__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__PSM_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PSM_STATE_ACQUISITION'.
enum
{
  ublox_ubx_msgs__msg__PSMStatus__PSM_STATE_ACQUISITION = 0
};

/// Constant 'PSM_STATE_TRACKING'.
enum
{
  ublox_ubx_msgs__msg__PSMStatus__PSM_STATE_TRACKING = 1
};

/// Constant 'PSM_STATE_POWER_OPTIMIZED_TRACKING'.
enum
{
  ublox_ubx_msgs__msg__PSMStatus__PSM_STATE_POWER_OPTIMIZED_TRACKING = 2
};

/// Constant 'PSM_STATE_INACTIVE'.
enum
{
  ublox_ubx_msgs__msg__PSMStatus__PSM_STATE_INACTIVE = 3
};

/// Struct defined in msg/PSMStatus in the package ublox_ubx_msgs.
/**
  * Power Save Mode for Status
 */
typedef struct ublox_ubx_msgs__msg__PSMStatus
{
  /// power save mode state
  uint8_t state;
} ublox_ubx_msgs__msg__PSMStatus;

// Struct for a sequence of ublox_ubx_msgs__msg__PSMStatus.
typedef struct ublox_ubx_msgs__msg__PSMStatus__Sequence
{
  ublox_ubx_msgs__msg__PSMStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__PSMStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__PSM_STATUS__STRUCT_H_
