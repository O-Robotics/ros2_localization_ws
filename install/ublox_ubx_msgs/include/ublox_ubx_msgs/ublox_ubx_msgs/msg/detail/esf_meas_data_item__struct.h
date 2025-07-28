// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/ESFMeasDataItem.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ESFMeasDataItem in the package ublox_ubx_msgs.
typedef struct ublox_ubx_msgs__msg__ESFMeasDataItem
{
  /// data
  uint32_t data_field;
  /// refer to manual
  uint8_t data_type;
} ublox_ubx_msgs__msg__ESFMeasDataItem;

// Struct for a sequence of ublox_ubx_msgs__msg__ESFMeasDataItem.
typedef struct ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence
{
  ublox_ubx_msgs__msg__ESFMeasDataItem * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__STRUCT_H_
