// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavCov.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__STRUCT_H_

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

/// Struct defined in msg/UBXNavCov in the package ublox_ubx_msgs.
/**
  * This message contains a UBX-NAV-COV (0x01 0x36) record clock solution
  *
  * It outputs the covariance matrices for the position and velocity
  * solutions in the topocentric coordinate system defined as the local-level North
  * (N), East (E), Down (D) frame. As the covariance matrices are symmetric, only
  * the upper triangular part is output.
 */
typedef struct ublox_ubx_msgs__msg__UBXNavCov
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// message version (0x00 for this version)
  uint8_t version;
  /// position covariance matrix validity flag
  bool pos_cor_valid;
  /// velocity covariance matrix validity flag
  bool vel_cor_valid;
  /// m^2 - Position covariance matric value p_NN
  float pos_cov_nn;
  /// m^2 - Position covariance matric value p_NE
  float pos_cov_ne;
  /// m^2 - Position covariance matric value p_ND
  float pos_cov_nd;
  /// m^2 - Position covariance matric value p_EE
  float pos_cov_ee;
  /// m^2 - Position covariance matric value p_ED
  float pos_cov_ed;
  /// m^2 - Position covariance matric value p_DD
  float pos_cov_dd;
  /// m^2/s^2 - Velocity covariance matric value v_NN
  float vel_cov_nn;
  /// m^2/s^2 - Velocity covariance matric value v_NE
  float vel_cov_ne;
  /// m^2/s^2 - Velocity covariance matric value v_ND
  float vel_cov_nd;
  /// m^2/s^2 - Velocity covariance matric value v_EE
  float vel_cov_ee;
  /// m^2/s^2 - Velocity covariance matric value v_ED
  float vel_cov_ed;
  /// m^2/s^2 - Velocity covariance matric value v_DD
  float vel_cov_dd;
} ublox_ubx_msgs__msg__UBXNavCov;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavCov.
typedef struct ublox_ubx_msgs__msg__UBXNavCov__Sequence
{
  ublox_ubx_msgs__msg__UBXNavCov * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavCov__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__STRUCT_H_
