// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXSecUniqid.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_UNIQID__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_UNIQID__STRUCT_H_

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

/// Struct defined in msg/UBXSecUniqid in the package ublox_ubx_msgs.
/**
  * UBX-SEC-UNIQID (0x27 0x03) - Unique chip ID
 */
typedef struct ublox_ubx_msgs__msg__UBXSecUniqid
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// Fields
  /// Message version
  uint8_t version;
  /// uint8 reserved_0    # Reserved
  /// Unique chip ID
  uint8_t unique_id[5];
} ublox_ubx_msgs__msg__UBXSecUniqid;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXSecUniqid.
typedef struct ublox_ubx_msgs__msg__UBXSecUniqid__Sequence
{
  ublox_ubx_msgs__msg__UBXSecUniqid * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXSecUniqid__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_UNIQID__STRUCT_H_
