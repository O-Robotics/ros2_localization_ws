// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavVelNED.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_vel_ned__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavVelNED & msg,
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

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: g_speed
  {
    out << "g_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.g_speed, out);
    out << ", ";
  }

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: s_acc
  {
    out << "s_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.s_acc, out);
    out << ", ";
  }

  // member: c_acc
  {
    out << "c_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.c_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavVelNED & msg,
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

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
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

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
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

  // member: c_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.c_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavVelNED & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavVelNED & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavVelNED & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavVelNED>()
{
  return "ublox_ubx_msgs::msg::UBXNavVelNED";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavVelNED>()
{
  return "ublox_ubx_msgs/msg/UBXNavVelNED";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavVelNED>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavVelNED>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavVelNED>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__TRAITS_HPP_
