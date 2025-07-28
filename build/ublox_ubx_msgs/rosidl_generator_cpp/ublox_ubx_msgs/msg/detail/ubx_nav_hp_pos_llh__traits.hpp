// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosLLH.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_llh__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavHPPosLLH & msg,
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

  // member: invalid_lon
  {
    out << "invalid_lon: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_lon, out);
    out << ", ";
  }

  // member: invalid_lat
  {
    out << "invalid_lat: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_lat, out);
    out << ", ";
  }

  // member: invalid_height
  {
    out << "invalid_height: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_height, out);
    out << ", ";
  }

  // member: invalid_hmsl
  {
    out << "invalid_hmsl: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_hmsl, out);
    out << ", ";
  }

  // member: invalid_lon_hp
  {
    out << "invalid_lon_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_lon_hp, out);
    out << ", ";
  }

  // member: invalid_lat_hp
  {
    out << "invalid_lat_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_lat_hp, out);
    out << ", ";
  }

  // member: invalid_height_hp
  {
    out << "invalid_height_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_height_hp, out);
    out << ", ";
  }

  // member: invalid_hmsl_hp
  {
    out << "invalid_hmsl_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_hmsl_hp, out);
    out << ", ";
  }

  // member: itow
  {
    out << "itow: ";
    rosidl_generator_traits::value_to_yaml(msg.itow, out);
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

  // member: lon_hp
  {
    out << "lon_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.lon_hp, out);
    out << ", ";
  }

  // member: lat_hp
  {
    out << "lat_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.lat_hp, out);
    out << ", ";
  }

  // member: height_hp
  {
    out << "height_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.height_hp, out);
    out << ", ";
  }

  // member: hmsl_hp
  {
    out << "hmsl_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.hmsl_hp, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavHPPosLLH & msg,
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

  // member: invalid_lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_lon: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_lon, out);
    out << "\n";
  }

  // member: invalid_lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_lat: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_lat, out);
    out << "\n";
  }

  // member: invalid_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_height: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_height, out);
    out << "\n";
  }

  // member: invalid_hmsl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_hmsl: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_hmsl, out);
    out << "\n";
  }

  // member: invalid_lon_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_lon_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_lon_hp, out);
    out << "\n";
  }

  // member: invalid_lat_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_lat_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_lat_hp, out);
    out << "\n";
  }

  // member: invalid_height_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_height_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_height_hp, out);
    out << "\n";
  }

  // member: invalid_hmsl_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invalid_hmsl_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.invalid_hmsl_hp, out);
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

  // member: lon_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.lon_hp, out);
    out << "\n";
  }

  // member: lat_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.lat_hp, out);
    out << "\n";
  }

  // member: height_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.height_hp, out);
    out << "\n";
  }

  // member: hmsl_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hmsl_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.hmsl_hp, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavHPPosLLH & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavHPPosLLH>()
{
  return "ublox_ubx_msgs::msg::UBXNavHPPosLLH";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavHPPosLLH>()
{
  return "ublox_ubx_msgs/msg/UBXNavHPPosLLH";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavHPPosLLH>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavHPPosLLH>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavHPPosLLH>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__TRAITS_HPP_
