// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/SBASStatusFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/sbas_status_flags__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SBASStatusFlags & msg,
  std::ostream & out)
{
  out << "{";
  // member: integrity_used
  {
    out << "integrity_used: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity_used, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SBASStatusFlags & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: integrity_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrity_used: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity_used, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SBASStatusFlags & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::SBASStatusFlags & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::SBASStatusFlags & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::SBASStatusFlags>()
{
  return "ublox_ubx_msgs::msg::SBASStatusFlags";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::SBASStatusFlags>()
{
  return "ublox_ubx_msgs/msg/SBASStatusFlags";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::SBASStatusFlags>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::SBASStatusFlags>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::SBASStatusFlags>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__TRAITS_HPP_
