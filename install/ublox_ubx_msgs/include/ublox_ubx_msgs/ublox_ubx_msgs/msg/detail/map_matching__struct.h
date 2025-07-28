// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/MapMatching.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__MAP_MATCHING__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__MAP_MATCHING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAP_MATCHING_NONE'.
enum
{
  ublox_ubx_msgs__msg__MapMatching__MAP_MATCHING_NONE = 0
};

/// Constant 'MAP_MATCHING_VALID_NOT_USED'.
enum
{
  ublox_ubx_msgs__msg__MapMatching__MAP_MATCHING_VALID_NOT_USED = 1
};

/// Constant 'MAP_MATCHING_VALID_AND_USED'.
enum
{
  ublox_ubx_msgs__msg__MapMatching__MAP_MATCHING_VALID_AND_USED = 2
};

/// Constant 'MAP_MATCHING_VALID_DEAD_RECKONING'.
enum
{
  ublox_ubx_msgs__msg__MapMatching__MAP_MATCHING_VALID_DEAD_RECKONING = 3
};

/// Struct defined in msg/MapMatching in the package ublox_ubx_msgs.
typedef struct ublox_ubx_msgs__msg__MapMatching
{
  /// map matching status
  uint8_t status;
} ublox_ubx_msgs__msg__MapMatching;

// Struct for a sequence of ublox_ubx_msgs__msg__MapMatching.
typedef struct ublox_ubx_msgs__msg__MapMatching__Sequence
{
  ublox_ubx_msgs__msg__MapMatching * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__MapMatching__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__MAP_MATCHING__STRUCT_H_
