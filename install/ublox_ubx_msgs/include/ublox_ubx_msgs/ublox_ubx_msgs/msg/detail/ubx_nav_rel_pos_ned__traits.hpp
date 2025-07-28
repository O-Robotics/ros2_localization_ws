// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavRelPosNED.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'carr_soln'
#include "ublox_ubx_msgs/msg/detail/carr_soln__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavRelPosNED & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: ref_station_id
  {
    out << "ref_station_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ref_station_id, out);
    out << ", ";
  }

  // member: itow
  {
    out << "itow: ";
    rosidl_generator_traits::value_to_yaml(msg.itow, out);
    out << ", ";
  }

  // member: rel_pos_n
  {
    out << "rel_pos_n: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_n, out);
    out << ", ";
  }

  // member: rel_pos_e
  {
    out << "rel_pos_e: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_e, out);
    out << ", ";
  }

  // member: rel_pos_d
  {
    out << "rel_pos_d: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_d, out);
    out << ", ";
  }

  // member: rel_pos_length
  {
    out << "rel_pos_length: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_length, out);
    out << ", ";
  }

  // member: rel_pos_heading
  {
    out << "rel_pos_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_heading, out);
    out << ", ";
  }

  // member: rel_pos_hp_n
  {
    out << "rel_pos_hp_n: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_hp_n, out);
    out << ", ";
  }

  // member: rel_pos_hp_e
  {
    out << "rel_pos_hp_e: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_hp_e, out);
    out << ", ";
  }

  // member: rel_pos_hp_d
  {
    out << "rel_pos_hp_d: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_hp_d, out);
    out << ", ";
  }

  // member: rel_pos_hp_length
  {
    out << "rel_pos_hp_length: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_hp_length, out);
    out << ", ";
  }

  // member: acc_n
  {
    out << "acc_n: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_n, out);
    out << ", ";
  }

  // member: acc_e
  {
    out << "acc_e: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_e, out);
    out << ", ";
  }

  // member: acc_d
  {
    out << "acc_d: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_d, out);
    out << ", ";
  }

  // member: acc_length
  {
    out << "acc_length: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_length, out);
    out << ", ";
  }

  // member: acc_heading
  {
    out << "acc_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_heading, out);
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

  // member: rel_pos_valid
  {
    out << "rel_pos_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_valid, out);
    out << ", ";
  }

  // member: carr_soln
  {
    out << "carr_soln: ";
    to_flow_style_yaml(msg.carr_soln, out);
    out << ", ";
  }

  // member: is_moving
  {
    out << "is_moving: ";
    rosidl_generator_traits::value_to_yaml(msg.is_moving, out);
    out << ", ";
  }

  // member: ref_pos_miss
  {
    out << "ref_pos_miss: ";
    rosidl_generator_traits::value_to_yaml(msg.ref_pos_miss, out);
    out << ", ";
  }

  // member: ref_obs_miss
  {
    out << "ref_obs_miss: ";
    rosidl_generator_traits::value_to_yaml(msg.ref_obs_miss, out);
    out << ", ";
  }

  // member: rel_pos_heading_valid
  {
    out << "rel_pos_heading_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_heading_valid, out);
    out << ", ";
  }

  // member: rel_pos_normalized
  {
    out << "rel_pos_normalized: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_normalized, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavRelPosNED & msg,
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

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: ref_station_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ref_station_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ref_station_id, out);
    out << "\n";
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

  // member: rel_pos_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_n: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_n, out);
    out << "\n";
  }

  // member: rel_pos_e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_e: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_e, out);
    out << "\n";
  }

  // member: rel_pos_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_d: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_d, out);
    out << "\n";
  }

  // member: rel_pos_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_length: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_length, out);
    out << "\n";
  }

  // member: rel_pos_heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_heading, out);
    out << "\n";
  }

  // member: rel_pos_hp_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_hp_n: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_hp_n, out);
    out << "\n";
  }

  // member: rel_pos_hp_e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_hp_e: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_hp_e, out);
    out << "\n";
  }

  // member: rel_pos_hp_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_hp_d: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_hp_d, out);
    out << "\n";
  }

  // member: rel_pos_hp_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_hp_length: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_hp_length, out);
    out << "\n";
  }

  // member: acc_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_n: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_n, out);
    out << "\n";
  }

  // member: acc_e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_e: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_e, out);
    out << "\n";
  }

  // member: acc_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_d: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_d, out);
    out << "\n";
  }

  // member: acc_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_length: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_length, out);
    out << "\n";
  }

  // member: acc_heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_heading, out);
    out << "\n";
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

  // member: rel_pos_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_valid, out);
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

  // member: is_moving
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_moving: ";
    rosidl_generator_traits::value_to_yaml(msg.is_moving, out);
    out << "\n";
  }

  // member: ref_pos_miss
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ref_pos_miss: ";
    rosidl_generator_traits::value_to_yaml(msg.ref_pos_miss, out);
    out << "\n";
  }

  // member: ref_obs_miss
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ref_obs_miss: ";
    rosidl_generator_traits::value_to_yaml(msg.ref_obs_miss, out);
    out << "\n";
  }

  // member: rel_pos_heading_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_heading_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_heading_valid, out);
    out << "\n";
  }

  // member: rel_pos_normalized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_normalized: ";
    rosidl_generator_traits::value_to_yaml(msg.rel_pos_normalized, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavRelPosNED & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavRelPosNED & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavRelPosNED>()
{
  return "ublox_ubx_msgs::msg::UBXNavRelPosNED";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavRelPosNED>()
{
  return "ublox_ubx_msgs/msg/UBXNavRelPosNED";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavRelPosNED>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<ublox_ubx_msgs::msg::CarrSoln>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavRelPosNED>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<ublox_ubx_msgs::msg::CarrSoln>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavRelPosNED>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__TRAITS_HPP_
