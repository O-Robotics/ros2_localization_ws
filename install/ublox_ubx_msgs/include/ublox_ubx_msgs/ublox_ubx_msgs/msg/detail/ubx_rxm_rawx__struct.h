// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXRxmRawx.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__STRUCT_H_

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
// Member 'rec_stat'
#include "ublox_ubx_msgs/msg/detail/rec_stat__struct.h"
// Member 'rawx_data'
#include "ublox_ubx_msgs/msg/detail/rawx_data__struct.h"

/// Struct defined in msg/UBXRxmRawx in the package ublox_ubx_msgs.
/**
  * UBXRxmRAWX.msg
  * UBX-RXM-RAWX (0x02 0x15) - Multi-GNSS raw measurements
 */
typedef struct ublox_ubx_msgs__msg__UBXRxmRawx
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// Fields
  /// s - Measurement time of week
  double rcv_tow;
  /// weeks - GPS week number
  uint16_t week;
  /// s - GPS leap seconds
  int8_t leap_s;
  /// Number of measurements
  uint8_t num_meas;
  /// Receiver tracking status bitfield
  ublox_ubx_msgs__msg__RecStat rec_stat;
  /// Message version
  uint8_t version;
  /// uint8 reserved0   # Reserved
  /// Repeated group of data for each satellite
  ublox_ubx_msgs__msg__RawxData__Sequence rawx_data;
} ublox_ubx_msgs__msg__UBXRxmRawx;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXRxmRawx.
typedef struct ublox_ubx_msgs__msg__UBXRxmRawx__Sequence
{
  ublox_ubx_msgs__msg__UBXRxmRawx * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXRxmRawx__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__STRUCT_H_
