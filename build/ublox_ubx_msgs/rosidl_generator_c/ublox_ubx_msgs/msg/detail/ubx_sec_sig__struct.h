// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXSecSig.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'JAM_UNKNOWN'.
/**
  * Constants for Jamming States
 */
enum
{
  ublox_ubx_msgs__msg__UBXSecSig__JAM_UNKNOWN = 0
};

/// Constant 'JAM_NO_JAMMING'.
enum
{
  ublox_ubx_msgs__msg__UBXSecSig__JAM_NO_JAMMING = 1
};

/// Constant 'JAM_WARNING'.
enum
{
  ublox_ubx_msgs__msg__UBXSecSig__JAM_WARNING = 2
};

/// Constant 'JAM_CRITICAL'.
enum
{
  ublox_ubx_msgs__msg__UBXSecSig__JAM_CRITICAL = 3
};

/// Constant 'SPF_UNKNOWN'.
/**
  * Constants for Spoofing States
 */
enum
{
  ublox_ubx_msgs__msg__UBXSecSig__SPF_UNKNOWN = 0
};

/// Constant 'SPF_NO_SPOOFING'.
enum
{
  ublox_ubx_msgs__msg__UBXSecSig__SPF_NO_SPOOFING = 1
};

/// Constant 'SPF_SPOOFING_INDICATED'.
enum
{
  ublox_ubx_msgs__msg__UBXSecSig__SPF_SPOOFING_INDICATED = 2
};

/// Constant 'SPF_SPOOFING_AFFIRMED'.
enum
{
  ublox_ubx_msgs__msg__UBXSecSig__SPF_SPOOFING_AFFIRMED = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/UBXSecSig in the package ublox_ubx_msgs.
/**
  * UBX-SEC-SIG (0x27 0x09) - Signal security information
 */
typedef struct ublox_ubx_msgs__msg__UBXSecSig
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// Fields
  /// Message version
  uint8_t version;
  /// uint8 reserved_0    # Reserved
  /// Information related to jamming/interference
  /// Flag indicates whether jamming/interference detection is enabled
  uint8_t jam_det_enabled;
  /// Jamming/interference state
  uint8_t jamming_state;
  /// uint8 reserved_1    # Reserved
  /// Information related to GNSS spoofing
  /// Flag indicates whether spoofing detection is enabled
  uint8_t spf_det_enabled;
  /// Spoofing state
  uint8_t spoofing_state;
} ublox_ubx_msgs__msg__UBXSecSig;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXSecSig.
typedef struct ublox_ubx_msgs__msg__UBXSecSig__Sequence
{
  ublox_ubx_msgs__msg__UBXSecSig * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXSecSig__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__STRUCT_H_
