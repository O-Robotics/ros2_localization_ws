// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/SBASSvData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SBASSvData in the package ublox_ubx_msgs.
/**
  * Information for each Satellite in SBAS
 */
typedef struct ublox_ubx_msgs__msg__SBASSvData
{
  /// SV ID
  uint8_t svid;
  /// Reserved
  uint8_t reserved_1;
  /// Monitoring status
  uint8_t udre;
  /// System (WAAS/EGNOS/...)
  uint8_t sv_sys;
  /// Services available, same as SERVICE
  uint8_t sv_service;
  /// Reserved
  uint8_t reserved_2;
  /// Pseudo Range correction in
  int16_t prc;
  /// Reserved
  uint8_t reserved_3[2];
  /// Ionosphere correction in
  int16_t ic;
} ublox_ubx_msgs__msg__SBASSvData;

// Struct for a sequence of ublox_ubx_msgs__msg__SBASSvData.
typedef struct ublox_ubx_msgs__msg__SBASSvData__Sequence
{
  ublox_ubx_msgs__msg__SBASSvData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__SBASSvData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__STRUCT_H_
