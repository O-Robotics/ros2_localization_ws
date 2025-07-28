// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavRelPosNED.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__STRUCT_H_

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
// Member 'carr_soln'
#include "ublox_ubx_msgs/msg/detail/carr_soln__struct.h"

/// Struct defined in msg/UBXNavRelPosNED in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-ODO (0x01 0x3c) record 
  * Relative position information in NED frame
 */
typedef struct ublox_ubx_msgs__msg__UBXNavRelPosNED
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// message version
  uint8_t version;
  /// reference station ID. Must be in the range 0..4096
  uint16_t ref_station_id;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// cm - North component of relative position vector
  int32_t rel_pos_n;
  /// cm - East component of relative position vector
  int32_t rel_pos_e;
  /// cm - Down component of relative position vector
  int32_t rel_pos_d;
  /// cm - Length of the relative position vector
  int32_t rel_pos_length;
  /// deg scale 1e-5 - Heading of the relative position vector
  int32_t rel_pos_heading;
  /// mm scale 0.1 - full HP North is given by relPosN + (relPosHPN * 1e-2)
  int8_t rel_pos_hp_n;
  /// mm scale 0.1 - full HP East is given by relPosE + (relPosHPE * 1e-2)
  int8_t rel_pos_hp_e;
  /// mm scale 0.1 - full HP Down is given by relPosD + (relPosHPD * 1e-2)
  int8_t rel_pos_hp_d;
  /// mm scale 0.1 - full HP length is given by relPosLength + (relPosHPLength * 1e-2)
  int8_t rel_pos_hp_length;
  /// mm scale 0.1 - Accuracy of relative position North Component
  uint32_t acc_n;
  /// mm scale 0.1 - Accuracy of relative position East Component
  uint32_t acc_e;
  /// mm scale 0.1 - Accuracy of relative position Down Component
  uint32_t acc_d;
  /// mm scale 0.1 - Accuracy of length of the relative position vector
  uint32_t acc_length;
  /// deg scale 1e-5 - Accuracy of heading of the relative position vector
  uint32_t acc_heading;
  /// A valid fix (i.e within DOP & accuracy masks)
  bool gnss_fix_ok;
  /// differential corrections were applied
  bool diff_soln;
  /// relative position components and accuracies are valid and, in moving base mode only, if baseline is valid
  bool rel_pos_valid;
  /// carrier phase range solution
  ublox_ubx_msgs__msg__CarrSoln carr_soln;
  /// if the received is operating in moving base mode
  bool is_moving;
  /// if extrapolated reference position was used to compute moving base solution this epoch.
  bool ref_pos_miss;
  /// if extrapolated reference observations were used to compute moving base solution this epoch.
  bool ref_obs_miss;
  /// relPosHeading is valid
  bool rel_pos_heading_valid;
  /// components of the relative position vector (including the high-precision parts) are normalized
  bool rel_pos_normalized;
} ublox_ubx_msgs__msg__UBXNavRelPosNED;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavRelPosNED.
typedef struct ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence
{
  ublox_ubx_msgs__msg__UBXNavRelPosNED * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavRelPosNED__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__STRUCT_H_
