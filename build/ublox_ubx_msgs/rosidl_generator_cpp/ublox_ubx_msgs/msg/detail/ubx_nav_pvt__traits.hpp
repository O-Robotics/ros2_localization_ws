// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPVT.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'gps_fix'
#include "ublox_ubx_msgs/msg/detail/gps_fix__traits.hpp"
// Member 'psm'
#include "ublox_ubx_msgs/msg/detail/psmpvt__traits.hpp"
// Member 'carr_soln'
#include "ublox_ubx_msgs/msg/detail/carr_soln__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavPVT & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: itow
  {
    out << "itow: ";
    rosidl_generator_traits::value_to_yaml(msg.itow, out);
    out << ", ";
  }

  // member: year
  {
    out << "year: ";
    rosidl_generator_traits::value_to_yaml(msg.year, out);
    out << ", ";
  }

  // member: month
  {
    out << "month: ";
    rosidl_generator_traits::value_to_yaml(msg.month, out);
    out << ", ";
  }

  // member: day
  {
    out << "day: ";
    rosidl_generator_traits::value_to_yaml(msg.day, out);
    out << ", ";
  }

  // member: hour
  {
    out << "hour: ";
    rosidl_generator_traits::value_to_yaml(msg.hour, out);
    out << ", ";
  }

  // member: min
  {
    out << "min: ";
    rosidl_generator_traits::value_to_yaml(msg.min, out);
    out << ", ";
  }

  // member: sec
  {
    out << "sec: ";
    rosidl_generator_traits::value_to_yaml(msg.sec, out);
    out << ", ";
  }

  // member: valid_date
  {
    out << "valid_date: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_date, out);
    out << ", ";
  }

  // member: valid_time
  {
    out << "valid_time: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_time, out);
    out << ", ";
  }

  // member: fully_resolved
  {
    out << "fully_resolved: ";
    rosidl_generator_traits::value_to_yaml(msg.fully_resolved, out);
    out << ", ";
  }

  // member: valid_mag
  {
    out << "valid_mag: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_mag, out);
    out << ", ";
  }

  // member: t_acc
  {
    out << "t_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.t_acc, out);
    out << ", ";
  }

  // member: nano
  {
    out << "nano: ";
    rosidl_generator_traits::value_to_yaml(msg.nano, out);
    out << ", ";
  }

  // member: gps_fix
  {
    out << "gps_fix: ";
    to_flow_style_yaml(msg.gps_fix, out);
    out << ", ";
  }

  // member: gnss_fix_ok
  {
    out << "gnss_fix_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_fix_ok, out);
    out << ", ";
  }

  // member: diff_soln
  {
    out << "diff_soln: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_soln, out);
    out << ", ";
  }

  // member: psm
  {
    out << "psm: ";
    to_flow_style_yaml(msg.psm, out);
    out << ", ";
  }

  // member: head_veh_valid
  {
    out << "head_veh_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.head_veh_valid, out);
    out << ", ";
  }

  // member: carr_soln
  {
    out << "carr_soln: ";
    to_flow_style_yaml(msg.carr_soln, out);
    out << ", ";
  }

  // member: confirmed_avail
  {
    out << "confirmed_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmed_avail, out);
    out << ", ";
  }

  // member: confirmed_date
  {
    out << "confirmed_date: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmed_date, out);
    out << ", ";
  }

  // member: confirmed_time
  {
    out << "confirmed_time: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmed_time, out);
    out << ", ";
  }

  // member: num_sv
  {
    out << "num_sv: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sv, out);
    out << ", ";
  }

  // member: lon
  {
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << ", ";
  }

  // member: lat
  {
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: hmsl
  {
    out << "hmsl: ";
    rosidl_generator_traits::value_to_yaml(msg.hmsl, out);
    out << ", ";
  }

  // member: h_acc
  {
    out << "h_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.h_acc, out);
    out << ", ";
  }

  // member: v_acc
  {
    out << "v_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.v_acc, out);
    out << ", ";
  }

  // member: vel_n
  {
    out << "vel_n: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_n, out);
    out << ", ";
  }

  // member: vel_e
  {
    out << "vel_e: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_e, out);
    out << ", ";
  }

  // member: vel_d
  {
    out << "vel_d: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_d, out);
    out << ", ";
  }

  // member: g_speed
  {
    out << "g_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.g_speed, out);
    out << ", ";
  }

  // member: head_mot
  {
    out << "head_mot: ";
    rosidl_generator_traits::value_to_yaml(msg.head_mot, out);
    out << ", ";
  }

  // member: s_acc
  {
    out << "s_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.s_acc, out);
    out << ", ";
  }

  // member: head_acc
  {
    out << "head_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.head_acc, out);
    out << ", ";
  }

  // member: p_dop
  {
    out << "p_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.p_dop, out);
    out << ", ";
  }

  // member: invalid_llh
  {
    out << "invalid_llh: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_llh, out);
    out << ", ";
  }

  // member: head_veh
  {
    out << "head_veh: ";
    rosidl_generator_traits::value_to_yaml(msg.head_veh, out);
    out << ", ";
  }

  // member: mag_dec
  {
    out << "mag_dec: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_dec, out);
    out << ", ";
  }

  // member: mag_acc
  {
    out << "mag_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavPVT & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: itow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "itow: ";
    rosidl_generator_traits::value_to_yaml(msg.itow, out);
    out << "\n";
  }

  // member: year
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "year: ";
    rosidl_generator_traits::value_to_yaml(msg.year, out);
    out << "\n";
  }

  // member: month
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "month: ";
    rosidl_generator_traits::value_to_yaml(msg.month, out);
    out << "\n";
  }

  // member: day
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "day: ";
    rosidl_generator_traits::value_to_yaml(msg.day, out);
    out << "\n";
  }

  // member: hour
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hour: ";
    rosidl_generator_traits::value_to_yaml(msg.hour, out);
    out << "\n";
  }

  // member: min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min: ";
    rosidl_generator_traits::value_to_yaml(msg.min, out);
    out << "\n";
  }

  // member: sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sec: ";
    rosidl_generator_traits::value_to_yaml(msg.sec, out);
    out << "\n";
  }

  // member: valid_date
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid_date: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_date, out);
    out << "\n";
  }

  // member: valid_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid_time: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_time, out);
    out << "\n";
  }

  // member: fully_resolved
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fully_resolved: ";
    rosidl_generator_traits::value_to_yaml(msg.fully_resolved, out);
    out << "\n";
  }

  // member: valid_mag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid_mag: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_mag, out);
    out << "\n";
  }

  // member: t_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.t_acc, out);
    out << "\n";
  }

  // member: nano
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nano: ";
    rosidl_generator_traits::value_to_yaml(msg.nano, out);
    out << "\n";
  }

  // member: gps_fix
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_fix:\n";
    to_block_style_yaml(msg.gps_fix, out, indentation + 2);
  }

  // member: gnss_fix_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gnss_fix_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_fix_ok, out);
    out << "\n";
  }

  // member: diff_soln
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diff_soln: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_soln, out);
    out << "\n";
  }

  // member: psm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "psm:\n";
    to_block_style_yaml(msg.psm, out, indentation + 2);
  }

  // member: head_veh_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "head_veh_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.head_veh_valid, out);
    out << "\n";
  }

  // member: carr_soln
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "carr_soln:\n";
    to_block_style_yaml(msg.carr_soln, out, indentation + 2);
  }

  // member: confirmed_avail
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confirmed_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmed_avail, out);
    out << "\n";
  }

  // member: confirmed_date
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confirmed_date: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmed_date, out);
    out << "\n";
  }

  // member: confirmed_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confirmed_time: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmed_time, out);
    out << "\n";
  }

  // member: num_sv
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sv: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sv, out);
    out << "\n";
  }

  // member: lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << "\n";
  }

  // member: lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: hmsl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hmsl: ";
    rosidl_generator_traits::value_to_yaml(msg.hmsl, out);
    out << "\n";
  }

  // member: h_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "h_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.h_acc, out);
    out << "\n";
  }

  // member: v_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.v_acc, out);
    out << "\n";
  }

  // member: vel_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_n: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_n, out);
    out << "\n";
  }

  // member: vel_e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_e: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_e, out);
    out << "\n";
  }

  // member: vel_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_d: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_d, out);
    out << "\n";
  }

  // member: g_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.g_speed, out);
    out << "\n";
  }

  // member: head_mot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "head_mot: ";
    rosidl_generator_traits::value_to_yaml(msg.head_mot, out);
    out << "\n";
  }

  // member: s_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.s_acc, out);
    out << "\n";
  }

  // member: head_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "head_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.head_acc, out);
    out << "\n";
  }

  // member: p_dop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "p_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.p_dop, out);
    out << "\n";
  }

  // member: invalid_llh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_llh: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_llh, out);
    out << "\n";
  }

  // member: head_veh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "head_veh: ";
    rosidl_generator_traits::value_to_yaml(msg.head_veh, out);
    out << "\n";
  }

  // member: mag_dec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mag_dec: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_dec, out);
    out << "\n";
  }

  // member: mag_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mag_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavPVT & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ublox_ubx_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ublox_ubx_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ublox_ubx_msgs::msg::UBXNavPVT & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavPVT & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavPVT>()
{
  return "ublox_ubx_msgs::msg::UBXNavPVT";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavPVT>()
{
  return "ublox_ubx_msgs/msg/UBXNavPVT";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavPVT>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<ublox_ubx_msgs::msg::CarrSoln>::value && has_fixed_size<ublox_ubx_msgs::msg::GpsFix>::value && has_fixed_size<ublox_ubx_msgs::msg::PSMPVT>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavPVT>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<ublox_ubx_msgs::msg::CarrSoln>::value && has_bounded_size<ublox_ubx_msgs::msg::GpsFix>::value && has_bounded_size<ublox_ubx_msgs::msg::PSMPVT>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavPVT>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__TRAITS_HPP_
