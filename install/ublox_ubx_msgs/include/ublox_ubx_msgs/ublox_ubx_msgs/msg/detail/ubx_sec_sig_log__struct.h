// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXSecSigLog.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG_LOG__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG_LOG__STRUCT_H_

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
// Member 'events'
#include "ublox_ubx_msgs/msg/detail/sig_log_event__struct.h"

/// Struct defined in msg/UBXSecSigLog in the package ublox_ubx_msgs.
/**
  * UBX-SEC-SIGLOG (0x27 0x10) - Signal security log
 */
typedef struct ublox_ubx_msgs__msg__UBXSecSigLog
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// Fields
  /// Message version
  uint8_t version;
  /// Number of events
  uint8_t num_events;
  /// uint8 reserved_0     # Reserved
  /// Repeated group of events
  /// Array of SigLogEvent messages
  ublox_ubx_msgs__msg__SigLogEvent__Sequence events;
} ublox_ubx_msgs__msg__UBXSecSigLog;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXSecSigLog.
typedef struct ublox_ubx_msgs__msg__UBXSecSigLog__Sequence
{
  ublox_ubx_msgs__msg__UBXSecSigLog * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXSecSigLog__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG_LOG__STRUCT_H_
