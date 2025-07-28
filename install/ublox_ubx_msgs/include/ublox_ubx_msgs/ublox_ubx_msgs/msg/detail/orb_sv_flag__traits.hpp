// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/OrbSVFlag.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/orb_sv_flag__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OrbSVFlag & msg,
  std::ostream & out)
{
  out << "{";
  // member: health
  {
    out << "health: ";
    rosidl_generator_traits::value_to_yaml(msg.health, out);
    out << ", ";
  }

  // member: visibility
  {
    out << "visibility: ";
    rosidl_generator_traits::value_to_yaml(msg.visibility, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OrbSVFlag & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: health
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "health: ";
    rosidl_generator_traits::value_to_yaml(msg.health, out);
    out << "\n";
  }

  // member: visibility
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visibility: ";
    rosidl_generator_traits::value_to_yaml(msg.visibility, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OrbSVFlag & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::OrbSVFlag & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::OrbSVFlag & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::OrbSVFlag>()
{
  return "ublox_ubx_msgs::msg::OrbSVFlag";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::OrbSVFlag>()
{
  return "ublox_ubx_msgs/msg/OrbSVFlag";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::OrbSVFlag>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::OrbSVFlag>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::OrbSVFlag>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__TRAITS_HPP_
