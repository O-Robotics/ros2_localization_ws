// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/OtherOrbInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__OTHER_ORB_INFO__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__OTHER_ORB_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ANO_AOP_USABILITY_UNKNOWN'.
/**
  * The usability period is unknown
 */
enum
{
  ublox_ubx_msgs__msg__OtherOrbInfo__ANO_AOP_USABILITY_UNKNOWN = 31
};

/// Constant 'ANO_AOP_USABILITY_OVER_30_DAYS'.
/**
  * The usability period is more than 30 days
 */
enum
{
  ublox_ubx_msgs__msg__OtherOrbInfo__ANO_AOP_USABILITY_OVER_30_DAYS = 30
};

/// Constant 'ANO_AOP_USABILITY_EXPIRED'.
/**
  * Data can no longer be used
 */
enum
{
  ublox_ubx_msgs__msg__OtherOrbInfo__ANO_AOP_USABILITY_EXPIRED = 0
};

/// Constant 'TYPE_NO_ORBIT_DATA'.
enum
{
  ublox_ubx_msgs__msg__OtherOrbInfo__TYPE_NO_ORBIT_DATA = 0
};

/// Constant 'TYPE_ASSISTNOW_OFFLINE'.
enum
{
  ublox_ubx_msgs__msg__OtherOrbInfo__TYPE_ASSISTNOW_OFFLINE = 1
};

/// Constant 'TYPE_ASSISTNOW_AUTONOMOUS'.
enum
{
  ublox_ubx_msgs__msg__OtherOrbInfo__TYPE_ASSISTNOW_AUTONOMOUS = 2
};

/// Struct defined in msg/OtherOrbInfo in the package ublox_ubx_msgs.
/**
  * Bit field structure for otherOrb
 */
typedef struct ublox_ubx_msgs__msg__OtherOrbInfo
{
  /// How long the receiver will be able to use the orbit data from now on.
  /// For 30 > n > 0, the usability period is between n-1 and n days.
  uint8_t ano_aop_usability;
  /// Type of orbit data
  uint8_t orb_type;
} ublox_ubx_msgs__msg__OtherOrbInfo;

// Struct for a sequence of ublox_ubx_msgs__msg__OtherOrbInfo.
typedef struct ublox_ubx_msgs__msg__OtherOrbInfo__Sequence
{
  ublox_ubx_msgs__msg__OtherOrbInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__OtherOrbInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__OTHER_ORB_INFO__STRUCT_H_
