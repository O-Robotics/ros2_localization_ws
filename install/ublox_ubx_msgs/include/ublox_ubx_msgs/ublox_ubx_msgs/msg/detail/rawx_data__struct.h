// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/RawxData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'trk_stat'
#include "ublox_ubx_msgs/msg/detail/trk_stat__struct.h"

/// Struct defined in msg/RawxData in the package ublox_ubx_msgs.
/**
  * RawxData.msg
  * UBX-RXM-RAWX (0x02 0x15) - Multi-GNSS raw measurements
 */
typedef struct ublox_ubx_msgs__msg__RawxData
{
  /// Fields
  /// m - Pseudorange measurement
  double pr_mes;
  /// cycles - Carrier phase measurement
  double cp_mes;
  /// Hz - Doppler measurement
  float do_mes;
  /// GNSS identifier
  uint8_t gnss_id;
  /// Satellite identifier
  uint8_t sv_id;
  /// Signal identifier
  uint8_t sig_id;
  /// Frequency slot for GLONASS
  uint8_t freq_id;
  /// ms - Carrier phase locktime counter
  uint16_t locktime;
  /// dBHz - Carrier-to-noise density ratio
  uint8_t c_no;
  /// 0.01*2^n m - Estimated pseudorange standard deviation
  uint8_t pr_stdev;
  /// 0.004 cycles - Estimated carrier phase standard deviation
  uint8_t cp_stdev;
  /// 0.002*2^n Hz - Estimated Doppler standard deviation
  uint8_t do_stdev;
  /// Tracking status bitfield
  ublox_ubx_msgs__msg__TrkStat trk_stat;
} ublox_ubx_msgs__msg__RawxData;

// Struct for a sequence of ublox_ubx_msgs__msg__RawxData.
typedef struct ublox_ubx_msgs__msg__RawxData__Sequence
{
  ublox_ubx_msgs__msg__RawxData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__RawxData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__STRUCT_H_
