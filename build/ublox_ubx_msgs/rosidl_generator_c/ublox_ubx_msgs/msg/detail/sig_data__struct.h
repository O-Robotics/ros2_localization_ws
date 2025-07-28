// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/SigData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'QUALITY_NO_SIGNAL'.
/**
  * Constants for Signal Quality Indicator
 */
enum
{
  ublox_ubx_msgs__msg__SigData__QUALITY_NO_SIGNAL = 0
};

/// Constant 'QUALITY_SEARCHING_SIGNAL'.
enum
{
  ublox_ubx_msgs__msg__SigData__QUALITY_SEARCHING_SIGNAL = 1
};

/// Constant 'QUALITY_SIGNAL_ACQUIRED'.
enum
{
  ublox_ubx_msgs__msg__SigData__QUALITY_SIGNAL_ACQUIRED = 2
};

/// Constant 'QUALITY_SIGNAL_UNUSABLE'.
enum
{
  ublox_ubx_msgs__msg__SigData__QUALITY_SIGNAL_UNUSABLE = 3
};

/// Constant 'QUALITY_CODE_LOCKED'.
enum
{
  ublox_ubx_msgs__msg__SigData__QUALITY_CODE_LOCKED = 4
};

/// Constant 'QUALITY_CODE_CARRIER_LOCKED'.
enum
{
  ublox_ubx_msgs__msg__SigData__QUALITY_CODE_CARRIER_LOCKED = 5
};

/// Constant 'CORR_NONE'.
/**
  * 6 and 7 are also considered as code and carrier locked and time synchronized
  * Constants for Correction Source
 */
enum
{
  ublox_ubx_msgs__msg__SigData__CORR_NONE = 0
};

/// Constant 'CORR_SBAS'.
enum
{
  ublox_ubx_msgs__msg__SigData__CORR_SBAS = 1
};

/// Constant 'CORR_BEIDOU'.
enum
{
  ublox_ubx_msgs__msg__SigData__CORR_BEIDOU = 2
};

/// Constant 'CORR_RTCM2'.
enum
{
  ublox_ubx_msgs__msg__SigData__CORR_RTCM2 = 3
};

/// Constant 'CORR_RTCM3_OSR'.
enum
{
  ublox_ubx_msgs__msg__SigData__CORR_RTCM3_OSR = 4
};

/// Constant 'CORR_RTCM3_SSR'.
enum
{
  ublox_ubx_msgs__msg__SigData__CORR_RTCM3_SSR = 5
};

/// Constant 'CORR_QZSS_SLAS'.
enum
{
  ublox_ubx_msgs__msg__SigData__CORR_QZSS_SLAS = 6
};

/// Constant 'CORR_SPARTN'.
enum
{
  ublox_ubx_msgs__msg__SigData__CORR_SPARTN = 7
};

/// Constant 'CORR_CLAS'.
enum
{
  ublox_ubx_msgs__msg__SigData__CORR_CLAS = 8
};

/// Constant 'IONO_NONE'.
/**
  * Constants for Ionospheric Model
 */
enum
{
  ublox_ubx_msgs__msg__SigData__IONO_NONE = 0
};

/// Constant 'IONO_KLOB_GPS'.
enum
{
  ublox_ubx_msgs__msg__SigData__IONO_KLOB_GPS = 1
};

/// Constant 'IONO_SBAS'.
enum
{
  ublox_ubx_msgs__msg__SigData__IONO_SBAS = 2
};

/// Constant 'IONO_KLOB_BEIDOU'.
enum
{
  ublox_ubx_msgs__msg__SigData__IONO_KLOB_BEIDOU = 3
};

// Include directives for member types
// Member 'sig_flags'
#include "ublox_ubx_msgs/msg/detail/sig_flags__struct.h"

/// Struct defined in msg/SigData in the package ublox_ubx_msgs.
/**
  * Information for each Signal in UBX-NAV-SIG
 */
typedef struct ublox_ubx_msgs__msg__SigData
{
  /// 8 is considered as Iono delay derived from dual frequency observations
  /// GNSS identifier
  uint8_t gnss_id;
  /// Satellite identifier
  uint8_t sv_id;
  /// New style signal identifier
  uint8_t sig_id;
  /// Frequency slot + 7 (GLONASS)
  uint8_t freq_id;
  /// Pseudorange residual (scaled by 0.1)
  int16_t pr_res;
  /// Carrier-to-noise density ratio
  uint8_t cno;
  /// Signal quality indicator
  uint8_t quality_ind;
  /// Correction source
  uint8_t corr_source;
  /// Ionospheric model used
  uint8_t iono_model;
  /// Signal related flags
  ublox_ubx_msgs__msg__SigFlags sig_flags;
} ublox_ubx_msgs__msg__SigData;

// Struct for a sequence of ublox_ubx_msgs__msg__SigData.
typedef struct ublox_ubx_msgs__msg__SigData__Sequence
{
  ublox_ubx_msgs__msg__SigData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__SigData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__STRUCT_H_
