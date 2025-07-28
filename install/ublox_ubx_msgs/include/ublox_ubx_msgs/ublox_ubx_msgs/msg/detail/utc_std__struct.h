// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UtcStd.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UTC_STD__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UTC_STD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NOT_AVAILABLE'.
enum
{
  ublox_ubx_msgs__msg__UtcStd__NOT_AVAILABLE = 0
};

/// Constant 'CRL'.
/**
  * Communications Research Labratory (CRL),Tokyo, Japan
 */
enum
{
  ublox_ubx_msgs__msg__UtcStd__CRL = 1
};

/// Constant 'NIST'.
/**
  * National Institute of Standards andTechnology (NIST)
 */
enum
{
  ublox_ubx_msgs__msg__UtcStd__NIST = 2
};

/// Constant 'USNO'.
/**
  * U.S. Naval Observatory (USNO)
 */
enum
{
  ublox_ubx_msgs__msg__UtcStd__USNO = 3
};

/// Constant 'BIPM'.
/**
  * International Bureau of Weights andMeasures (BIPM)
 */
enum
{
  ublox_ubx_msgs__msg__UtcStd__BIPM = 4
};

/// Constant 'EURO'.
/**
  * European laboratories
 */
enum
{
  ublox_ubx_msgs__msg__UtcStd__EURO = 5
};

/// Constant 'SU'.
/**
  * Former Soviet Union
 */
enum
{
  ublox_ubx_msgs__msg__UtcStd__SU = 6
};

/// Constant 'NTSC'.
/**
  * National Time Service Center (NTSC) China
 */
enum
{
  ublox_ubx_msgs__msg__UtcStd__NTSC = 7
};

/// Constant 'UTC_UNKNOWN'.
/**
  * Unknown time source
 */
enum
{
  ublox_ubx_msgs__msg__UtcStd__UTC_UNKNOWN = 15
};

/// Struct defined in msg/UtcStd in the package ublox_ubx_msgs.
typedef struct ublox_ubx_msgs__msg__UtcStd
{
  /// uc standard id
  uint8_t id;
} ublox_ubx_msgs__msg__UtcStd;

// Struct for a sequence of ublox_ubx_msgs__msg__UtcStd.
typedef struct ublox_ubx_msgs__msg__UtcStd__Sequence
{
  ublox_ubx_msgs__msg__UtcStd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UtcStd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UTC_STD__STRUCT_H_
