// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/OrbSVInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/orb_sv_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'sv_flag'
#include "ublox_ubx_msgs/msg/detail/orb_sv_flag__traits.hpp"
// Member 'eph'
#include "ublox_ubx_msgs/msg/detail/orb_eph_info__traits.hpp"
// Member 'alm'
#include "ublox_ubx_msgs/msg/detail/orb_alm_info__traits.hpp"
// Member 'other_orb'
#include "ublox_ubx_msgs/msg/detail/other_orb_info__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OrbSVInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: gnss_id
  {
    out << "gnss_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_id, out);
    out << ", ";
  }

  // member: sv_id
  {
    out << "sv_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_id, out);
    out << ", ";
  }

  // member: sv_flag
  {
    out << "sv_flag: ";
    to_flow_style_yaml(msg.sv_flag, out);
    out << ", ";
  }

  // member: eph
  {
    out << "eph: ";
    to_flow_style_yaml(msg.eph, out);
    out << ", ";
  }

  // member: alm
  {
    out << "alm: ";
    to_flow_style_yaml(msg.alm, out);
    out << ", ";
  }

  // member: other_orb
  {
    out << "other_orb: ";
    to_flow_style_yaml(msg.other_orb, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OrbSVInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: gnss_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gnss_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_id, out);
    out << "\n";
  }

  // member: sv_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sv_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_id, out);
    out << "\n";
  }

  // member: sv_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sv_flag:\n";
    to_block_style_yaml(msg.sv_flag, out, indentation + 2);
  }

  // member: eph
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eph:\n";
    to_block_style_yaml(msg.eph, out, indentation + 2);
  }

  // member: alm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alm:\n";
    to_block_style_yaml(msg.alm, out, indentation + 2);
  }

  // member: other_orb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "other_orb:\n";
    to_block_style_yaml(msg.other_orb, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OrbSVInfo & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::OrbSVInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::OrbSVInfo & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::OrbSVInfo>()
{
  return "ublox_ubx_msgs::msg::OrbSVInfo";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::OrbSVInfo>()
{
  return "ublox_ubx_msgs/msg/OrbSVInfo";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::OrbSVInfo>
  : std::integral_constant<bool, has_fixed_size<ublox_ubx_msgs::msg::OrbAlmInfo>::value && has_fixed_size<ublox_ubx_msgs::msg::OrbEphInfo>::value && has_fixed_size<ublox_ubx_msgs::msg::OrbSVFlag>::value && has_fixed_size<ublox_ubx_msgs::msg::OtherOrbInfo>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::OrbSVInfo>
  : std::integral_constant<bool, has_bounded_size<ublox_ubx_msgs::msg::OrbAlmInfo>::value && has_bounded_size<ublox_ubx_msgs::msg::OrbEphInfo>::value && has_bounded_size<ublox_ubx_msgs::msg::OrbSVFlag>::value && has_bounded_size<ublox_ubx_msgs::msg::OtherOrbInfo>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::OrbSVInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__TRAITS_HPP_
