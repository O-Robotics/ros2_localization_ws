// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavDOP.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_dop__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavDOP & msg,
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

  // member: g_dop
  {
    out << "g_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.g_dop, out);
    out << ", ";
  }

  // member: p_dop
  {
    out << "p_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.p_dop, out);
    out << ", ";
  }

  // member: t_dop
  {
    out << "t_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.t_dop, out);
    out << ", ";
  }

  // member: v_dop
  {
    out << "v_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.v_dop, out);
    out << ", ";
  }

  // member: h_dop
  {
    out << "h_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.h_dop, out);
    out << ", ";
  }

  // member: n_dop
  {
    out << "n_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.n_dop, out);
    out << ", ";
  }

  // member: e_dop
  {
    out << "e_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.e_dop, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavDOP & msg,
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

  // member: g_dop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.g_dop, out);
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

  // member: t_dop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.t_dop, out);
    out << "\n";
  }

  // member: v_dop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.v_dop, out);
    out << "\n";
  }

  // member: h_dop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "h_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.h_dop, out);
    out << "\n";
  }

  // member: n_dop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "n_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.n_dop, out);
    out << "\n";
  }

  // member: e_dop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "e_dop: ";
    rosidl_generator_traits::value_to_yaml(msg.e_dop, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavDOP & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavDOP & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavDOP & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavDOP>()
{
  return "ublox_ubx_msgs::msg::UBXNavDOP";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavDOP>()
{
  return "ublox_ubx_msgs/msg/UBXNavDOP";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavDOP>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavDOP>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavDOP>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__TRAITS_HPP_
