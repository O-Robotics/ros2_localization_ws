// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavClock.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_clock__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavClock & msg,
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

  // member: clk_b
  {
    out << "clk_b: ";
    rosidl_generator_traits::value_to_yaml(msg.clk_b, out);
    out << ", ";
  }

  // member: clk_d
  {
    out << "clk_d: ";
    rosidl_generator_traits::value_to_yaml(msg.clk_d, out);
    out << ", ";
  }

  // member: t_acc
  {
    out << "t_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.t_acc, out);
    out << ", ";
  }

  // member: f_acc
  {
    out << "f_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.f_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavClock & msg,
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

  // member: clk_b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "clk_b: ";
    rosidl_generator_traits::value_to_yaml(msg.clk_b, out);
    out << "\n";
  }

  // member: clk_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "clk_d: ";
    rosidl_generator_traits::value_to_yaml(msg.clk_d, out);
    out << "\n";
  }

  // member: t_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.t_acc, out);
    out << "\n";
  }

  // member: f_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.f_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavClock & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavClock & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavClock & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavClock>()
{
  return "ublox_ubx_msgs::msg::UBXNavClock";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavClock>()
{
  return "ublox_ubx_msgs/msg/UBXNavClock";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavClock>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavClock>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavClock>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__TRAITS_HPP_
