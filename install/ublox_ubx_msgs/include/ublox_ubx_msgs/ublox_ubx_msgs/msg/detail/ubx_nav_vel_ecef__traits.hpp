// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavVelECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_vel_ecef__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavVelECEF & msg,
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

  // member: ecef_vx
  {
    out << "ecef_vx: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_vx, out);
    out << ", ";
  }

  // member: ecef_vy
  {
    out << "ecef_vy: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_vy, out);
    out << ", ";
  }

  // member: ecef_vz
  {
    out << "ecef_vz: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_vz, out);
    out << ", ";
  }

  // member: s_acc
  {
    out << "s_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.s_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavVelECEF & msg,
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

  // member: ecef_vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ecef_vx: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_vx, out);
    out << "\n";
  }

  // member: ecef_vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ecef_vy: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_vy, out);
    out << "\n";
  }

  // member: ecef_vz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ecef_vz: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_vz, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavVelECEF & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavVelECEF & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavVelECEF & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavVelECEF>()
{
  return "ublox_ubx_msgs::msg::UBXNavVelECEF";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavVelECEF>()
{
  return "ublox_ubx_msgs/msg/UBXNavVelECEF";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavVelECEF>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavVelECEF>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavVelECEF>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__TRAITS_HPP_
