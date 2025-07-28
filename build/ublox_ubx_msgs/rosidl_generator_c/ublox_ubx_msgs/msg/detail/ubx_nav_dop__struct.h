// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavDOP.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__STRUCT_H_

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

/// Struct defined in msg/UBXNavDOP in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-DOP (0x01 0x04) record dilution of precision
 */
typedef struct ublox_ubx_msgs__msg__UBXNavDOP
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// 0.01 - Geometric DOP
  uint32_t g_dop;
  /// 0.01 - Position DOP
  uint32_t p_dop;
  /// 0.01 - Time DOP
  uint32_t t_dop;
  /// 0.01 - Vertical DOP
  uint32_t v_dop;
  /// 0.01 - Horizontal DOP
  uint32_t h_dop;
  /// 0.01 - Northing DOP
  uint32_t n_dop;
  /// 0.01 - Easting DOP
  uint32_t e_dop;
} ublox_ubx_msgs__msg__UBXNavDOP;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavDOP.
typedef struct ublox_ubx_msgs__msg__UBXNavDOP__Sequence
{
  ublox_ubx_msgs__msg__UBXNavDOP * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavDOP__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__STRUCT_H_
