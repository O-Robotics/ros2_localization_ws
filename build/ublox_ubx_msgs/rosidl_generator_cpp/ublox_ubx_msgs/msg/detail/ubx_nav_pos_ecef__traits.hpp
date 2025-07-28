// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPosECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_pos_ecef__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavPosECEF & msg,
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

  // member: ecef_x
  {
    out << "ecef_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_x, out);
    out << ", ";
  }

  // member: ecef_y
  {
    out << "ecef_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_y, out);
    out << ", ";
  }

  // member: ecef_z
  {
    out << "ecef_z: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_z, out);
    out << ", ";
  }

  // member: p_acc
  {
    out << "p_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.p_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavPosECEF & msg,
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

  // member: ecef_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ecef_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_x, out);
    out << "\n";
  }

  // member: ecef_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ecef_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_y, out);
    out << "\n";
  }

  // member: ecef_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ecef_z: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_z, out);
    out << "\n";
  }

  // member: p_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "p_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.p_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavPosECEF & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavPosECEF & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavPosECEF & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavPosECEF>()
{
  return "ublox_ubx_msgs::msg::UBXNavPosECEF";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavPosECEF>()
{
  return "ublox_ubx_msgs/msg/UBXNavPosECEF";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavPosECEF>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavPosECEF>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavPosECEF>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__TRAITS_HPP_
