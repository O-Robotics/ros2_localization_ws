// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/RecStat.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__REC_STAT__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__REC_STAT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RecStat in the package ublox_ubx_msgs.
/**
  * RecStat.msg
  * UBX-RXM-RAWX (0x02 0x15) - Receiver tracking status bitfield
 */
typedef struct ublox_ubx_msgs__msg__RecStat
{
  /// Fields
  /// Leap seconds have been determined
  bool leap_sec;
  /// Clock reset applied
  bool clk_reset;
} ublox_ubx_msgs__msg__RecStat;

// Struct for a sequence of ublox_ubx_msgs__msg__RecStat.
typedef struct ublox_ubx_msgs__msg__RecStat__Sequence
{
  ublox_ubx_msgs__msg__RecStat * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__RecStat__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__REC_STAT__STRUCT_H_
