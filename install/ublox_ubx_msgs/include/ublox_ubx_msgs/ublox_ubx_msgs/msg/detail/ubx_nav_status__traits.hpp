// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'gps_fix'
#include "ublox_ubx_msgs/msg/detail/gps_fix__traits.hpp"
// Member 'map_matching'
#include "ublox_ubx_msgs/msg/detail/map_matching__traits.hpp"
// Member 'psm'
#include "ublox_ubx_msgs/msg/detail/psm_status__traits.hpp"
// Member 'spoof_det'
#include "ublox_ubx_msgs/msg/detail/spoof_det__traits.hpp"
// Member 'carr_soln'
#include "ublox_ubx_msgs/msg/detail/carr_soln__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavStatus & msg,
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

  // member: gps_fix
  {
    out << "gps_fix: ";
    to_flow_style_yaml(msg.gps_fix, out);
    out << ", ";
  }

  // member: gps_fix_ok
  {
    out << "gps_fix_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_fix_ok, out);
    out << ", ";
  }

  // member: diff_soln
  {
    out << "diff_soln: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_soln, out);
    out << ", ";
  }

  // member: wkn_set
  {
    out << "wkn_set: ";
    rosidl_generator_traits::value_to_yaml(msg.wkn_set, out);
    out << ", ";
  }

  // member: tow_set
  {
    out << "tow_set: ";
    rosidl_generator_traits::value_to_yaml(msg.tow_set, out);
    out << ", ";
  }

  // member: diff_corr
  {
    out << "diff_corr: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_corr, out);
    out << ", ";
  }

  // member: carr_soln_valid
  {
    out << "carr_soln_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.carr_soln_valid, out);
    out << ", ";
  }

  // member: map_matching
  {
    out << "map_matching: ";
    to_flow_style_yaml(msg.map_matching, out);
    out << ", ";
  }

  // member: psm
  {
    out << "psm: ";
    to_flow_style_yaml(msg.psm, out);
    out << ", ";
  }

  // member: spoof_det
  {
    out << "spoof_det: ";
    to_flow_style_yaml(msg.spoof_det, out);
    out << ", ";
  }

  // member: carr_soln
  {
    out << "carr_soln: ";
    to_flow_style_yaml(msg.carr_soln, out);
    out << ", ";
  }

  // member: ttff
  {
    out << "ttff: ";
    rosidl_generator_traits::value_to_yaml(msg.ttff, out);
    out << ", ";
  }

  // member: msss
  {
    out << "msss: ";
    rosidl_generator_traits::value_to_yaml(msg.msss, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavStatus & msg,
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

  // member: gps_fix
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_fix:\n";
    to_block_style_yaml(msg.gps_fix, out, indentation + 2);
  }

  // member: gps_fix_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_fix_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_fix_ok, out);
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

  // member: wkn_set
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wkn_set: ";
    rosidl_generator_traits::value_to_yaml(msg.wkn_set, out);
    out << "\n";
  }

  // member: tow_set
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tow_set: ";
    rosidl_generator_traits::value_to_yaml(msg.tow_set, out);
    out << "\n";
  }

  // member: diff_corr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diff_corr: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_corr, out);
    out << "\n";
  }

  // member: carr_soln_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "carr_soln_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.carr_soln_valid, out);
    out << "\n";
  }

  // member: map_matching
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_matching:\n";
    to_block_style_yaml(msg.map_matching, out, indentation + 2);
  }

  // member: psm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "psm:\n";
    to_block_style_yaml(msg.psm, out, indentation + 2);
  }

  // member: spoof_det
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spoof_det:\n";
    to_block_style_yaml(msg.spoof_det, out, indentation + 2);
  }

  // member: carr_soln
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "carr_soln:\n";
    to_block_style_yaml(msg.carr_soln, out, indentation + 2);
  }

  // member: ttff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ttff: ";
    rosidl_generator_traits::value_to_yaml(msg.ttff, out);
    out << "\n";
  }

  // member: msss
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "msss: ";
    rosidl_generator_traits::value_to_yaml(msg.msss, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavStatus & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavStatus & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavStatus>()
{
  return "ublox_ubx_msgs::msg::UBXNavStatus";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavStatus>()
{
  return "ublox_ubx_msgs/msg/UBXNavStatus";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<ublox_ubx_msgs::msg::CarrSoln>::value && has_fixed_size<ublox_ubx_msgs::msg::GpsFix>::value && has_fixed_size<ublox_ubx_msgs::msg::MapMatching>::value && has_fixed_size<ublox_ubx_msgs::msg::PSMStatus>::value && has_fixed_size<ublox_ubx_msgs::msg::SpoofDet>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<ublox_ubx_msgs::msg::CarrSoln>::value && has_bounded_size<ublox_ubx_msgs::msg::GpsFix>::value && has_bounded_size<ublox_ubx_msgs::msg::MapMatching>::value && has_bounded_size<ublox_ubx_msgs::msg::PSMStatus>::value && has_bounded_size<ublox_ubx_msgs::msg::SpoofDet>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__TRAITS_HPP_
