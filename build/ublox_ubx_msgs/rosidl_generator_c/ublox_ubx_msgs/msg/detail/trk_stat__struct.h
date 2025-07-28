// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/TrkStat.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TrkStat in the package ublox_ubx_msgs.
/**
  * TrkStat.msg
  * UBX-RXM-RAWX (0x02 0x15) - Tracking status bitfield for Multi-GNSS raw measurements
 */
typedef struct ublox_ubx_msgs__msg__TrkStat
{
  /// Fields
  /// Pseudorange valid
  bool pr_valid;
  /// Carrier phase valid
  bool cp_valid;
  /// Half cycle valid
  bool half_cyc;
  /// Half cycle subtracted from phase
  bool sub_half_cyc;
} ublox_ubx_msgs__msg__TrkStat;

// Struct for a sequence of ublox_ubx_msgs__msg__TrkStat.
typedef struct ublox_ubx_msgs__msg__TrkStat__Sequence
{
  ublox_ubx_msgs__msg__TrkStat * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__TrkStat__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__STRUCT_H_
