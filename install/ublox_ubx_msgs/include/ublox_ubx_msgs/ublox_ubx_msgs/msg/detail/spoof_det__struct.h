// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/SpoofDet.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SPOOF_DET__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SPOOF_DET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SPOOF_DET_UNKNOWN'.
enum
{
  ublox_ubx_msgs__msg__SpoofDet__SPOOF_DET_UNKNOWN = 0
};

/// Constant 'SPOOF_DET_NO_SPOOFING'.
enum
{
  ublox_ubx_msgs__msg__SpoofDet__SPOOF_DET_NO_SPOOFING = 1
};

/// Constant 'SPOOF_DET_SPOOFING'.
enum
{
  ublox_ubx_msgs__msg__SpoofDet__SPOOF_DET_SPOOFING = 2
};

/// Constant 'SPOOF_DET_MULTIPLE_SPOOFING'.
enum
{
  ublox_ubx_msgs__msg__SpoofDet__SPOOF_DET_MULTIPLE_SPOOFING = 3
};

/// Struct defined in msg/SpoofDet in the package ublox_ubx_msgs.
typedef struct ublox_ubx_msgs__msg__SpoofDet
{
  /// spoofing det state
  uint8_t state;
} ublox_ubx_msgs__msg__SpoofDet;

// Struct for a sequence of ublox_ubx_msgs__msg__SpoofDet.
typedef struct ublox_ubx_msgs__msg__SpoofDet__Sequence
{
  ublox_ubx_msgs__msg__SpoofDet * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__SpoofDet__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SPOOF_DET__STRUCT_H_
