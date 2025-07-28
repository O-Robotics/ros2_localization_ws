// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXEsfStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'WT_INIT_STATUS_OFF'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__WT_INIT_STATUS_OFF = 0
};

/// Constant 'WT_INIT_STATUS_INITIALIZING'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__WT_INIT_STATUS_INITIALIZING = 1
};

/// Constant 'WT_INIT_STATUS_INITIALIZED'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__WT_INIT_STATUS_INITIALIZED = 2
};

/// Constant 'MBT_ALG_STATUS_OFF'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__MBT_ALG_STATUS_OFF = 0
};

/// Constant 'MBT_ALG_STATUS_INITIALIZING'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__MBT_ALG_STATUS_INITIALIZING = 1
};

/// Constant 'MBT_ALG_STATUS_INITIALIZED0'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__MBT_ALG_STATUS_INITIALIZED0 = 2
};

/// Constant 'MBT_ALG_STATUS_INITIALIZED1'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__MBT_ALG_STATUS_INITIALIZED1 = 3
};

/// Constant 'INS_INIT_STATUS_OFF'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__INS_INIT_STATUS_OFF = 0
};

/// Constant 'INS_INIT_STATUS_INITIALIZING'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__INS_INIT_STATUS_INITIALIZING = 1
};

/// Constant 'INS_INIT_STATUS_INITIALIZED'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__INS_INIT_STATUS_INITIALIZED = 2
};

/// Constant 'IMU_INIT_STATUS_OFF'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__IMU_INIT_STATUS_OFF = 0
};

/// Constant 'IMU_INIT_STATUS_INITIALIZING'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__IMU_INIT_STATUS_INITIALIZING = 1
};

/// Constant 'IMU_INIT_STATUS_INITIALIZED'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__IMU_INIT_STATUS_INITIALIZED = 2
};

/// Constant 'FUSION_MODE_INITIALIZATION'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__FUSION_MODE_INITIALIZATION = 0
};

/// Constant 'FUSION_MODE_WORKING'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__FUSION_MODE_WORKING = 1
};

/// Constant 'FUSION_MODE_SUSPENDED'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__FUSION_MODE_SUSPENDED = 2
};

/// Constant 'FUSION_MODE_DISABLED'.
enum
{
  ublox_ubx_msgs__msg__UBXEsfStatus__FUSION_MODE_DISABLED = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'sensor_statuses'
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__struct.h"

/// Struct defined in msg/UBXEsfStatus in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-ESF-STATUS (0x10 0x10) record
  * External sesnor fusion status
 */
typedef struct ublox_ubx_msgs__msg__UBXEsfStatus
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// message version (0x02 for this version)
  uint8_t version;
  /// Wheel tick factor initialization status (0: off, 1:initializing, 2: initialized).
  uint8_t wt_init_status;
  /// Automatic IMU-mount alignment status (0: off, 1:initializing, 2: initialized, 3: initialized).
  uint8_t mnt_alg_status;
  /// INS initialization status (0: off, 1: initializing, 2:initialized).
  uint8_t ins_init_status;
  /// IMU initialization status (0: off, 1: initializing, 2:initialized).
  uint8_t imu_init_status;
  /// 0: initialisation, 1 Fusion, 2 Suspended, 3 Disabled
  uint8_t fusion_mode;
  /// number of sensors
  uint8_t num_sens;
  ublox_ubx_msgs__msg__ESFSensorStatus__Sequence sensor_statuses;
} ublox_ubx_msgs__msg__UBXEsfStatus;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXEsfStatus.
typedef struct ublox_ubx_msgs__msg__UBXEsfStatus__Sequence
{
  ublox_ubx_msgs__msg__UBXEsfStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXEsfStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__STRUCT_H_
