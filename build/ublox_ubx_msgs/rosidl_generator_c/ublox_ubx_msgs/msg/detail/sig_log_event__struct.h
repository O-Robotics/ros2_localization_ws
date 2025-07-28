// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/SigLogEvent.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DETECTION_SIMULATED_SIGNAL'.
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__DETECTION_SIMULATED_SIGNAL = 0
};

/// Constant 'DETECTION_ABNORMAL_SIGNAL'.
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__DETECTION_ABNORMAL_SIGNAL = 1
};

/// Constant 'DETECTION_INS_GNSS_MISMATCH'.
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__DETECTION_INS_GNSS_MISMATCH = 2
};

/// Constant 'DETECTION_ABRUPT_CHANGES'.
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__DETECTION_ABRUPT_CHANGES = 3
};

/// Constant 'DETECTION_BROADBAND_JAMMING'.
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__DETECTION_BROADBAND_JAMMING = 4
};

/// Constant 'DETECTION_NARROWBAND_JAMMING'.
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__DETECTION_NARROWBAND_JAMMING = 5
};

/// Constant 'EVENT_STARTED'.
/**
  * Constants for Event Types
 */
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__EVENT_STARTED = 0
};

/// Constant 'EVENT_STOPPED'.
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__EVENT_STOPPED = 1
};

/// Constant 'EVENT_TRIGGERED'.
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__EVENT_TRIGGERED = 2
};

/// Constant 'EVENT_TIMED_OUT'.
enum
{
  ublox_ubx_msgs__msg__SigLogEvent__EVENT_TIMED_OUT = 3
};

/// Struct defined in msg/SigLogEvent in the package ublox_ubx_msgs.
/**
  * Constants for Detection Types
 */
typedef struct ublox_ubx_msgs__msg__SigLogEvent
{
  /// Fields
  /// s - Seconds elapsed since this event. Special value 0xFFFFFFFF: more than 45 days
  uint32_t time_elapsed;
  /// Type of the spoofing or jamming detection
  uint8_t detection_type;
  /// Type of the event
  uint8_t event_type;
} ublox_ubx_msgs__msg__SigLogEvent;

// Struct for a sequence of ublox_ubx_msgs__msg__SigLogEvent.
typedef struct ublox_ubx_msgs__msg__SigLogEvent__Sequence
{
  ublox_ubx_msgs__msg__SigLogEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__SigLogEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__STRUCT_H_
