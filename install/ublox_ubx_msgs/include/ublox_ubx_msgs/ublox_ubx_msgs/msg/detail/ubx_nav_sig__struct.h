// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavSig.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SIG__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SIG__STRUCT_H_

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
// Member 'sig_data'
#include "ublox_ubx_msgs/msg/detail/sig_data__struct.h"

/// Struct defined in msg/UBXNavSig in the package ublox_ubx_msgs.
/**
  * UBX-NAV-SIG (0x01 0x43) - Signal information
 */
typedef struct ublox_ubx_msgs__msg__UBXNavSig
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// Message version
  uint8_t version;
  /// Number of signals
  uint8_t num_sigs;
  /// Reserved
  uint8_t reserved_0[2];
  /// Repeated group of data for each signal
  ublox_ubx_msgs__msg__SigData__Sequence sig_data;
} ublox_ubx_msgs__msg__UBXNavSig;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavSig.
typedef struct ublox_ubx_msgs__msg__UBXNavSig__Sequence
{
  ublox_ubx_msgs__msg__UBXNavSig * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavSig__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SIG__STRUCT_H_
