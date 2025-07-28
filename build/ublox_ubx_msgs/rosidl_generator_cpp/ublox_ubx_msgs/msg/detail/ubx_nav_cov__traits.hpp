// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavCov.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_cov__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavCov & msg,
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

  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: pos_cor_valid
  {
    out << "pos_cor_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cor_valid, out);
    out << ", ";
  }

  // member: vel_cor_valid
  {
    out << "vel_cor_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cor_valid, out);
    out << ", ";
  }

  // member: pos_cov_nn
  {
    out << "pos_cov_nn: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_nn, out);
    out << ", ";
  }

  // member: pos_cov_ne
  {
    out << "pos_cov_ne: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_ne, out);
    out << ", ";
  }

  // member: pos_cov_nd
  {
    out << "pos_cov_nd: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_nd, out);
    out << ", ";
  }

  // member: pos_cov_ee
  {
    out << "pos_cov_ee: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_ee, out);
    out << ", ";
  }

  // member: pos_cov_ed
  {
    out << "pos_cov_ed: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_ed, out);
    out << ", ";
  }

  // member: pos_cov_dd
  {
    out << "pos_cov_dd: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_dd, out);
    out << ", ";
  }

  // member: vel_cov_nn
  {
    out << "vel_cov_nn: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_nn, out);
    out << ", ";
  }

  // member: vel_cov_ne
  {
    out << "vel_cov_ne: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_ne, out);
    out << ", ";
  }

  // member: vel_cov_nd
  {
    out << "vel_cov_nd: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_nd, out);
    out << ", ";
  }

  // member: vel_cov_ee
  {
    out << "vel_cov_ee: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_ee, out);
    out << ", ";
  }

  // member: vel_cov_ed
  {
    out << "vel_cov_ed: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_ed, out);
    out << ", ";
  }

  // member: vel_cov_dd
  {
    out << "vel_cov_dd: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_dd, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavCov & msg,
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

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: pos_cor_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_cor_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cor_valid, out);
    out << "\n";
  }

  // member: vel_cor_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_cor_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cor_valid, out);
    out << "\n";
  }

  // member: pos_cov_nn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_cov_nn: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_nn, out);
    out << "\n";
  }

  // member: pos_cov_ne
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_cov_ne: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_ne, out);
    out << "\n";
  }

  // member: pos_cov_nd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_cov_nd: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_nd, out);
    out << "\n";
  }

  // member: pos_cov_ee
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_cov_ee: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_ee, out);
    out << "\n";
  }

  // member: pos_cov_ed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_cov_ed: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_ed, out);
    out << "\n";
  }

  // member: pos_cov_dd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_cov_dd: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_cov_dd, out);
    out << "\n";
  }

  // member: vel_cov_nn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_cov_nn: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_nn, out);
    out << "\n";
  }

  // member: vel_cov_ne
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_cov_ne: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_ne, out);
    out << "\n";
  }

  // member: vel_cov_nd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_cov_nd: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_nd, out);
    out << "\n";
  }

  // member: vel_cov_ee
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_cov_ee: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_ee, out);
    out << "\n";
  }

  // member: vel_cov_ed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_cov_ed: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_ed, out);
    out << "\n";
  }

  // member: vel_cov_dd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_cov_dd: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_cov_dd, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavCov & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavCov & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavCov & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavCov>()
{
  return "ublox_ubx_msgs::msg::UBXNavCov";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavCov>()
{
  return "ublox_ubx_msgs/msg/UBXNavCov";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavCov>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavCov>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavCov>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__TRAITS_HPP_
