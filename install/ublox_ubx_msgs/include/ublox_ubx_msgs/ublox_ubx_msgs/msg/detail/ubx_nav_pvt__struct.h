// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/UBXNavPVT.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__STRUCT_H_

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
// Member 'psm'
#include "ublox_ubx_msgs/msg/detail/psmpvt__struct.h"
// Member 'carr_soln'
#include "ublox_ubx_msgs/msg/detail/carr_soln__struct.h"

/// Struct defined in msg/UBXNavPVT in the package ublox_ubx_msgs.
/**
  * this message contains a UBX-NAV-PVT (0x01 0x07) record
  * Navigation position velocity time solution
 */
typedef struct ublox_ubx_msgs__msg__UBXNavPVT
{
  /// Header timestamp should be acquisition time
  std_msgs__msg__Header header;
  /// ms - GPS Time of week of the navigation epoch
  uint32_t itow;
  /// y - year utc
  uint16_t year;
  /// month - month utc
  uint8_t month;
  /// d -day utc
  uint8_t day;
  /// h - hour utc
  uint8_t hour;
  /// min - min utc
  uint8_t min;
  /// s - sec utc
  uint8_t sec;
  /// valid UTC date
  bool valid_date;
  /// valid UTC time of day
  bool valid_time;
  /// UTC Time of day has been fully resolved (no seconds uncertainty)
  bool fully_resolved;
  /// valid magnetic declination
  bool valid_mag;
  /// ns - time accuracy estimate (utc)
  uint32_t t_acc;
  /// ns - fraction of second, range -1e9 to 1e9 (utc)
  int32_t nano;
  /// GNSSFix type
  ublox_ubx_msgs__msg__GpsFix gps_fix;
  /// valid fix (ie within DOP & accruacy masks)
  bool gnss_fix_ok;
  /// differential corrections were applied
  bool diff_soln;
  /// power save mode state
  ublox_ubx_msgs__msg__PSMPVT psm;
  /// heading of vehicle is valid, only set if the receiver is in sensor fusion mode
  bool head_veh_valid;
  /// carrier phase rnage solution status
  ublox_ubx_msgs__msg__CarrSoln carr_soln;
  /// information about UTC date and time of day validity confirmation is available
  bool confirmed_avail;
  /// UTC Date validity could be confirmed
  bool confirmed_date;
  /// UTC Time of Day validity could be confirmed
  bool confirmed_time;
  /// Number of satellites used in Nav Solution
  uint8_t num_sv;
  /// deg scale ie-7 - longitude
  int32_t lon;
  /// deg scale ie-7 - latitude
  int32_t lat;
  /// mm - Height above ellipsoid
  int32_t height;
  /// mm - Height above mean sea level
  int32_t hmsl;
  /// mm - Horizontal accuracy estimate
  uint32_t h_acc;
  /// mm - Vertical accuracy estimate
  uint32_t v_acc;
  /// mm/s - NED north velocity
  int32_t vel_n;
  /// mm/s - NED east velocity
  int32_t vel_e;
  /// mm/s - NED down velocity
  int32_t vel_d;
  /// mm/s - ground speed (2-D)
  int32_t g_speed;
  /// deg scale ie-5 - heading of motion (2-D)
  int32_t head_mot;
  /// mm/s - speed accuracy estimate
  uint32_t s_acc;
  /// deg scale ie-5 - heading accruracy estime (both motion and vehicle)
  uint32_t head_acc;
  /// scale 0.01 - Position DOP
  uint16_t p_dop;
  /// invalid lon, lat, height, hmsl
  bool invalid_llh;
  /// deg scale 1e-5 - Heading of vehicle (2-D), this is only valid when head_veh_valid is set,
  /// otherwise the output is set to the heading of motion
  int32_t head_veh;
  /// deg scale 1e-2 - Magnetic declination. Only supported in ADR 4.10 and later
  int16_t mag_dec;
  /// deg scale ie-2 - Mangetic declination accuracy. Only supported in ADR 4.10 and later
  uint16_t mag_acc;
} ublox_ubx_msgs__msg__UBXNavPVT;

// Struct for a sequence of ublox_ubx_msgs__msg__UBXNavPVT.
typedef struct ublox_ubx_msgs__msg__UBXNavPVT__Sequence
{
  ublox_ubx_msgs__msg__UBXNavPVT * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__UBXNavPVT__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__STRUCT_H_
