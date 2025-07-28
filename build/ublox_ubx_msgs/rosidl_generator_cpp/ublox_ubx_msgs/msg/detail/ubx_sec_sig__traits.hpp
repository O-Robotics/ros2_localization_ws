// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXSecSig.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_sec_sig__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXSecSig & msg,
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

  // member: jam_det_enabled
  {
    out << "jam_det_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.jam_det_enabled, out);
    out << ", ";
  }

  // member: jamming_state
  {
    out << "jamming_state: ";
    rosidl_generator_traits::value_to_yaml(msg.jamming_state, out);
    out << ", ";
  }

  // member: spf_det_enabled
  {
    out << "spf_det_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.spf_det_enabled, out);
    out << ", ";
  }

  // member: spoofing_state
  {
    out << "spoofing_state: ";
    rosidl_generator_traits::value_to_yaml(msg.spoofing_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXSecSig & msg,
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

  // member: jam_det_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jam_det_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.jam_det_enabled, out);
    out << "\n";
  }

  // member: jamming_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jamming_state: ";
    rosidl_generator_traits::value_to_yaml(msg.jamming_state, out);
    out << "\n";
  }

  // member: spf_det_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spf_det_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.spf_det_enabled, out);
    out << "\n";
  }

  // member: spoofing_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spoofing_state: ";
    rosidl_generator_traits::value_to_yaml(msg.spoofing_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXSecSig & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXSecSig & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXSecSig & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXSecSig>()
{
  return "ublox_ubx_msgs::msg::UBXSecSig";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXSecSig>()
{
  return "ublox_ubx_msgs/msg/UBXSecSig";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXSecSig>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXSecSig>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXSecSig>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__TRAITS_HPP_
