// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__STRUCT_H_

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
// Member 'gps_fix'
#include "ublox_ubx_msgs/msg/detail/gps_fix__struct.h"
// Member 'map_matching'
#include "ublox_ubx_msgs/msg/detail/map_matching__struct.h"
// Member 'psm'
#include "ublox_ubx_msgs/msg/detail/psm_status__struct.h"
// Member 'spoof_det'
#include "ublox_ubx_msgs/msg/detail/spoof_det__struct.h"
// Member 'carr_soln'
#include "ublox_ubx_msgs/msg/detail/carr_soln__struct.h"

/// Struct defined in msg/UBXNavStatus in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-STATUS (0x01 0x03) record
 */
typedef struct ublox_ubx_msgs__msg__UBXNavStatus
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// gps fix
  ublox_ubx_msgs__msg__GpsFix gps_fix;
  /// navigation status flags
  /// position and velocity valid and within DOP and ACCMasks
  bool gps_fix_ok;
  /// differential corrections were applied
  bool diff_soln;
  /// week number valid
  bool wkn_set;
  /// time of week valid
  bool tow_set;
  /// navigation fix status information
  /// differential corrections available
  bool diff_corr;
  /// valid carrSoln
  bool carr_soln_valid;
  /// map matching status
  ublox_ubx_msgs__msg__MapMatching map_matching;
  /// further information about navigation output
  /// power save mode state
  ublox_ubx_msgs__msg__PSMStatus psm;
  ublox_ubx_msgs__msg__SpoofDet spoof_det;
  /// Carrier phase solution status
  ublox_ubx_msgs__msg__CarrSoln carr_soln;
  /// time to first fix (milliseond time tag)
  uint32_t ttff;
  /// milliseconds since Startup/Reset
  uint32_t msss;
} ublox_ubx_msgs__msg__UBXNavStatus;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavStatus.
typedef struct ublox_ubx_msgs__msg__UBXNavStatus__Sequence
{
  ublox_ubx_msgs__msg__UBXNavStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__STRUCT_H_
