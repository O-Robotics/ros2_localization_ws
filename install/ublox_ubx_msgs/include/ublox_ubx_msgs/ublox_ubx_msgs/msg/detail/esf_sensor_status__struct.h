// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/ESFSensorStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CALIB_STATUS_NOT_CALIBRATED'.
enum
{
  ublox_ubx_msgs__msg__ESFSensorStatus__CALIB_STATUS_NOT_CALIBRATED = 0
};

/// Constant 'CALIB_STATUS_CALIBRATING'.
enum
{
  ublox_ubx_msgs__msg__ESFSensorStatus__CALIB_STATUS_CALIBRATING = 1
};

/// Constant 'CALIB_STATUS_CALIBRATED0'.
enum
{
  ublox_ubx_msgs__msg__ESFSensorStatus__CALIB_STATUS_CALIBRATED0 = 2
};

/// Constant 'CALIB_STATUS_CALIBRATED1'.
enum
{
  ublox_ubx_msgs__msg__ESFSensorStatus__CALIB_STATUS_CALIBRATED1 = 3
};

/// Constant 'TIME_STATUS_NO_DATA'.
enum
{
  ublox_ubx_msgs__msg__ESFSensorStatus__TIME_STATUS_NO_DATA = 0
};

/// Constant 'TIME_STATUS_FIRST_BYTE_USED'.
enum
{
  ublox_ubx_msgs__msg__ESFSensorStatus__TIME_STATUS_FIRST_BYTE_USED = 1
};

/// Constant 'TIME_STATUS_TTAG_PROVIDED'.
enum
{
  ublox_ubx_msgs__msg__ESFSensorStatus__TIME_STATUS_TTAG_PROVIDED = 3
};

/// Struct defined in msg/ESFSensorStatus in the package ublox_ubx_msgs.
typedef struct ublox_ubx_msgs__msg__ESFSensorStatus
{
  /// see sensor data type in the integration manual
  uint8_t sensor_data_type;
  /// if set, sensor data is used for the current fusion solution
  bool used;
  /// if set, sensor is set up but not used for sensor fusion
  bool ready;
  /// 0x00: sensor not calibrated 0x01: sensor is calibrating,
  /// 0x10/0x11 sensor is calibrated. Good dead recking only possible
  /// when all used sensors are calibrated
  uint8_t calib_status;
  /// 0x00: No Data, 0x01: Reception of the first byte used to tag measurement
  /// 0x11: Time tag provided with the data
  uint8_t time_status;
  /// observation frequency
  uint8_t freq;
  /// bad measurements detected
  bool fault_bad_meas;
  /// bad measurement time-tages detected
  bool fault_bad_ttag;
  /// Missing or time-misaligned measurements detected
  bool fault_missing_meas;
  /// high measurement noise-level detected
  bool fault_noisy_meas;
} ublox_ubx_msgs__msg__ESFSensorStatus;

// Struct for a sequence of ublox_ubx_msgs__msg__ESFSensorStatus.
typedef struct ublox_ubx_msgs__msg__ESFSensorStatus__Sequence
{
  ublox_ubx_msgs__msg__ESFSensorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__ESFSensorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__STRUCT_H_
