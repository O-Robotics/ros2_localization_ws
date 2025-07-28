// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosLLH.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__STRUCT_H_

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

/// Struct defined in msg/UBXNavHPPosLLH in the package ublox_ubx_msgs.
/**
  * UBXNavHPPosLLH.msg
  * this message contains a UBX-NAV-HPPOSLLH (0x01 0x14) record 
 */
typedef struct ublox_ubx_msgs__msg__UBXNavHPPosLLH
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// message version
  uint8_t version;
  bool invalid_lon;
  bool invalid_lat;
  bool invalid_height;
  /// invalid height above mean sea level
  bool invalid_hmsl;
  bool invalid_lon_hp;
  bool invalid_lat_hp;
  bool invalid_height_hp;
  /// invalid height above mean sea level
  bool invalid_hmsl_hp;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// deg scale 1e-7 - longitude
  int32_t lon;
  /// deg scale 1e-7 - latitude
  int32_t lat;
  /// mm - height above ellipsoid
  int32_t height;
  /// mm - height above mean sea level
  int32_t hmsl;
  /// deg scale 1e-9 - Precision longitude in deg * 1e-7 = lon +(lonHp * 1e-2)
  int8_t lon_hp;
  /// deg scale 1e-9 - Precise latitude in deg  *  1e-7  =  lat  +(latHp * 1e-2)
  int8_t lat_hp;
  /// mm scale 0.1 - Precise height in mm = height + (heightHp * 0.1)
  int8_t height_hp;
  /// mm scale 0.1 - Precise height in mm =hMSL + (hMSLHp * 0.1)
  int8_t hmsl_hp;
  /// mm scale 0.1 - horizontal accuracy estimate
  uint32_t h_acc;
  /// mm scale 0.1 - vertical accuracy estimate
  uint32_t v_acc;
} ublox_ubx_msgs__msg__UBXNavHPPosLLH;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavHPPosLLH.
typedef struct ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence
{
  ublox_ubx_msgs__msg__UBXNavHPPosLLH * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavHPPosLLH__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__STRUCT_H_
