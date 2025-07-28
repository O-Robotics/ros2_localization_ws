// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/OrbEphInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/orb_eph_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OrbEphInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: eph_usability
  {
    out << "eph_usability: ";
    rosidl_generator_traits::value_to_yaml(msg.eph_usability, out);
    out << ", ";
  }

  // member: eph_source
  {
    out << "eph_source: ";
    rosidl_generator_traits::value_to_yaml(msg.eph_source, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OrbEphInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: eph_usability
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eph_usability: ";
    rosidl_generator_traits::value_to_yaml(msg.eph_usability, out);
    out << "\n";
  }

  // member: eph_source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eph_source: ";
    rosidl_generator_traits::value_to_yaml(msg.eph_source, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OrbEphInfo & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::OrbEphInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::OrbEphInfo & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::OrbEphInfo>()
{
  return "ublox_ubx_msgs::msg::OrbEphInfo";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::OrbEphInfo>()
{
  return "ublox_ubx_msgs/msg/OrbEphInfo";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::OrbEphInfo>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::OrbEphInfo>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::OrbEphInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__TRAITS_HPP_
