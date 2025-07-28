// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXRxmMeasx.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TOW_NOT_SET'.
/**
  * Constants for TOW Set
 */
enum
{
  ublox_ubx_msgs__msg__UBXRxmMeasx__TOW_NOT_SET = 0
};

/// Constant 'TOW_SET'.
enum
{
  ublox_ubx_msgs__msg__UBXRxmMeasx__TOW_SET = 1
};

/// Constant 'TOW_SET2'.
enum
{
  ublox_ubx_msgs__msg__UBXRxmMeasx__TOW_SET2 = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'sv_data'
#include "ublox_ubx_msgs/msg/detail/measx_data__struct.h"

/// Struct defined in msg/UBXRxmMeasx in the package ublox_ubx_msgs.
/**
  * UBX-RXM-MEASX (0x02 0x14) - Satellite measurements for RRLP
 */
typedef struct ublox_ubx_msgs__msg__UBXRxmMeasx
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// Fields
  /// Message version
  uint8_t version;
  /// uint8 reserved_0   # Reserved
  /// ms - GPS measurement reference time
  uint32_t gps_tow;
  /// ms - GLONASS measurement reference time
  uint32_t glo_tow;
  /// ms - BeiDou measurement reference time
  uint32_t bds_tow;
  /// uint8 reserved_1   # Reserved
  /// ms - QZSS measurement reference time
  uint32_t qzss_tow;
  /// 2^-4 ms - GPS measurement reference time accuracy
  uint16_t gps_tow_acc;
  /// 2^-4 ms - GLONASS measurement reference time accuracy
  uint16_t glo_tow_acc;
  /// 2^-4 ms - BeiDou measurement reference time accuracy
  uint16_t bds_tow_acc;
  /// uint8 reserved_2   # Reserved
  /// 2^-4 ms - QZSS measurement reference time accuracy
  uint16_t qzss_tow_acc;
  /// Number of satellites in repeated block
  uint8_t num_sv;
  /// Flags (refer to constants for TOW Set)
  uint8_t flags;
  /// uint8 reserved_3   # Reserved
  /// Repeated group of data for each satellite
  ublox_ubx_msgs__msg__MeasxData__Sequence sv_data;
} ublox_ubx_msgs__msg__UBXRxmMeasx;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXRxmMeasx.
typedef struct ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence
{
  ublox_ubx_msgs__msg__UBXRxmMeasx * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXRxmMeasx__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__STRUCT_H_
