// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavHPPosECEF & msg,
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

  // member: ecef_x_hp
  {
    out << "ecef_x_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_x_hp, out);
    out << ", ";
  }

  // member: ecef_y_hp
  {
    out << "ecef_y_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_y_hp, out);
    out << ", ";
  }

  // member: ecef_z_hp
  {
    out << "ecef_z_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_z_hp, out);
    out << ", ";
  }

  // member: invalid_ecef_x
  {
    out << "invalid_ecef_x: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_x, out);
    out << ", ";
  }

  // member: invalid_ecef_y
  {
    out << "invalid_ecef_y: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_y, out);
    out << ", ";
  }

  // member: invalid_ecef_z
  {
    out << "invalid_ecef_z: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_z, out);
    out << ", ";
  }

  // member: invalid_ecef_x_hp
  {
    out << "invalid_ecef_x_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_x_hp, out);
    out << ", ";
  }

  // member: invalid_ecef_y_hp
  {
    out << "invalid_ecef_y_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_y_hp, out);
    out << ", ";
  }

  // member: invalid_ecef_z_hp
  {
    out << "invalid_ecef_z_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_z_hp, out);
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
  const UBXNavHPPosECEF & msg,
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

  // member: ecef_x_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ecef_x_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_x_hp, out);
    out << "\n";
  }

  // member: ecef_y_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ecef_y_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_y_hp, out);
    out << "\n";
  }

  // member: ecef_z_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ecef_z_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.ecef_z_hp, out);
    out << "\n";
  }

  // member: invalid_ecef_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_ecef_x: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_x, out);
    out << "\n";
  }

  // member: invalid_ecef_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_ecef_y: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_y, out);
    out << "\n";
  }

  // member: invalid_ecef_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_ecef_z: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_z, out);
    out << "\n";
  }

  // member: invalid_ecef_x_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_ecef_x_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_x_hp, out);
    out << "\n";
  }

  // member: invalid_ecef_y_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_ecef_y_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_y_hp, out);
    out << "\n";
  }

  // member: invalid_ecef_z_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_ecef_z_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_ecef_z_hp, out);
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

inline std::string to_yaml(const UBXNavHPPosECEF & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavHPPosECEF>()
{
  return "ublox_ubx_msgs::msg::UBXNavHPPosECEF";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavHPPosECEF>()
{
  return "ublox_ubx_msgs/msg/UBXNavHPPosECEF";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavHPPosECEF>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavHPPosECEF>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavHPPosECEF>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__TRAITS_HPP_
