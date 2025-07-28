// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/GpsFix.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__GPS_FIX__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__GPS_FIX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GPS_NO_FIX'.
enum
{
  ublox_ubx_msgs__msg__GpsFix__GPS_NO_FIX = 0
};

/// Constant 'GPS_DEAD_RECKONING_ONLY'.
enum
{
  ublox_ubx_msgs__msg__GpsFix__GPS_DEAD_RECKONING_ONLY = 1
};

/// Constant 'GPS_FIX_2D'.
enum
{
  ublox_ubx_msgs__msg__GpsFix__GPS_FIX_2D = 2
};

/// Constant 'GPS_FIX_3D'.
enum
{
  ublox_ubx_msgs__msg__GpsFix__GPS_FIX_3D = 3
};

/// Constant 'GPS_PLUS_DEAD_RECKONING'.
enum
{
  ublox_ubx_msgs__msg__GpsFix__GPS_PLUS_DEAD_RECKONING = 4
};

/// Constant 'GPS_TIME_ONLY'.
enum
{
  ublox_ubx_msgs__msg__GpsFix__GPS_TIME_ONLY = 5
};

/// Struct defined in msg/GpsFix in the package ublox_ubx_msgs.
typedef struct ublox_ubx_msgs__msg__GpsFix
{
  /// gps fix type
  uint8_t fix_type;
} ublox_ubx_msgs__msg__GpsFix;

// Struct for a sequence of ublox_ubx_msgs__msg__GpsFix.
typedef struct ublox_ubx_msgs__msg__GpsFix__Sequence
{
  ublox_ubx_msgs__msg__GpsFix * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__GpsFix__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__GPS_FIX__STRUCT_H_
