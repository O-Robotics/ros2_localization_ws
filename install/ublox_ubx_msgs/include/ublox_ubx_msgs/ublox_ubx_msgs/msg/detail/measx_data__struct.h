// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/MeasxData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MPATH_NOT_MEASURED'.
/**
  * Constants for Multipath Indicators
  * Not measured
 */
enum
{
  ublox_ubx_msgs__msg__MeasxData__MPATH_NOT_MEASURED = 0
};

/// Constant 'MPATH_LOW'.
/**
  * Low multipath
 */
enum
{
  ublox_ubx_msgs__msg__MeasxData__MPATH_LOW = 1
};

/// Constant 'MPATH_MEDIUM'.
/**
  * Medium multipath
 */
enum
{
  ublox_ubx_msgs__msg__MeasxData__MPATH_MEDIUM = 2
};

/// Constant 'MPATH_HIGH'.
/**
  * High multipath
 */
enum
{
  ublox_ubx_msgs__msg__MeasxData__MPATH_HIGH = 3
};

/// Struct defined in msg/MeasxData in the package ublox_ubx_msgs.
/**
  * MeasxData - Satellite measurements data for UBX-RXM-MEASX (0x02 0x14)
 */
typedef struct ublox_ubx_msgs__msg__MeasxData
{
  /// Fields
  /// GNSS identifier
  uint8_t gnss_id;
  /// Satellite identifier
  uint8_t sv_id;
  /// dBHz - Carrier noise ratio
  uint8_t c_no;
  /// Multipath indicator (refer to constants)
  uint8_t mpath_indic;
  /// 0.04 m/s - Doppler measurement in m/s
  int32_t doppler_ms;
  /// 0.2 Hz - Doppler measurement in Hz
  int32_t doppler_hz;
  /// Whole value of the code phase measurement
  uint16_t whole_chips;
  /// Fractional value of the code phase measurement
  uint16_t frac_chips;
  /// 2^-21 ms - Code phase in ms
  uint32_t code_phase;
  /// ms - Integer part of the code phase
  uint8_t int_code_phase;
  /// Pseudorange RMS error index
  uint8_t pseu_range_rms_err;
} ublox_ubx_msgs__msg__MeasxData;

// Struct for a sequence of ublox_ubx_msgs__msg__MeasxData.
typedef struct ublox_ubx_msgs__msg__MeasxData__Sequence
{
  ublox_ubx_msgs__msg__MeasxData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__MeasxData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__STRUCT_H_
