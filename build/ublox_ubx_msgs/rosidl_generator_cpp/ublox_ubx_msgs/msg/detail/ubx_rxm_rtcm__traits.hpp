// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmRTCM.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rtcm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXRxmRTCM & msg,
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

  // member: crc_failed
  {
    out << "crc_failed: ";
    rosidl_generator_traits::value_to_yaml(msg.crc_failed, out);
    out << ", ";
  }

  // member: msg_used
  {
    out << "msg_used: ";
    rosidl_generator_traits::value_to_yaml(msg.msg_used, out);
    out << ", ";
  }

  // member: sub_type
  {
    out << "sub_type: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_type, out);
    out << ", ";
  }

  // member: ref_station
  {
    out << "ref_station: ";
    rosidl_generator_traits::value_to_yaml(msg.ref_station, out);
    out << ", ";
  }

  // member: msg_type
  {
    out << "msg_type: ";
    rosidl_generator_traits::value_to_yaml(msg.msg_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXRxmRTCM & msg,
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

  // member: crc_failed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "crc_failed: ";
    rosidl_generator_traits::value_to_yaml(msg.crc_failed, out);
    out << "\n";
  }

  // member: msg_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "msg_used: ";
    rosidl_generator_traits::value_to_yaml(msg.msg_used, out);
    out << "\n";
  }

  // member: sub_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sub_type: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_type, out);
    out << "\n";
  }

  // member: ref_station
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ref_station: ";
    rosidl_generator_traits::value_to_yaml(msg.ref_station, out);
    out << "\n";
  }

  // member: msg_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "msg_type: ";
    rosidl_generator_traits::value_to_yaml(msg.msg_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXRxmRTCM & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXRxmRTCM & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXRxmRTCM & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXRxmRTCM>()
{
  return "ublox_ubx_msgs::msg::UBXRxmRTCM";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXRxmRTCM>()
{
  return "ublox_ubx_msgs/msg/UBXRxmRTCM";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXRxmRTCM>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXRxmRTCM>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXRxmRTCM>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__TRAITS_HPP_
