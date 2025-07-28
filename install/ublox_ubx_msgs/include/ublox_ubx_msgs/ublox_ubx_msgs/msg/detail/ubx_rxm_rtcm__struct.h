// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXRxmRTCM.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__STRUCT_H_

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

/// Struct defined in msg/UBXRxmRTCM in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-RXM-RTCM (0x02 0x32) record
  * RTCM Input status
  * This message shows info on a received RTCM input message. It is output upon successful parsing of an RTCM
  * input message, irrespective of whether the RTCM message is supported or not by the receiver.
 */
typedef struct ublox_ubx_msgs__msg__UBXRxmRTCM
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// message version (0x02 for this version)
  uint8_t version;
  /// 0 When RTCM received & CRC Ok, 1 when failed
  bool crc_failed;
  /// 2 = RTCM msg used sucessfully, 1 = Not Used, 0 = do not know
  uint8_t msg_used;
  /// message subtype, only applicable to ublox proprietary RTCM 4072
  uint16_t sub_type;
  /// reference station ID refer interface description
  uint16_t ref_station;
  /// message type
  uint16_t msg_type;
} ublox_ubx_msgs__msg__UBXRxmRTCM;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXRxmRTCM.
typedef struct ublox_ubx_msgs__msg__UBXRxmRTCM__Sequence
{
  ublox_ubx_msgs__msg__UBXRxmRTCM * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXRxmRTCM__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__STRUCT_H_
