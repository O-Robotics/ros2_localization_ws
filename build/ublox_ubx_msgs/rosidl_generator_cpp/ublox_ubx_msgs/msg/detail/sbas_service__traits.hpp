// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/SBASService.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/sbas_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SBASService & msg,
  std::ostream & out)
{
  out << "{";
  // member: ranging
  {
    out << "ranging: ";
    rosidl_generator_traits::value_to_yaml(msg.ranging, out);
    out << ", ";
  }

  // member: corrections
  {
    out << "corrections: ";
    rosidl_generator_traits::value_to_yaml(msg.corrections, out);
    out << ", ";
  }

  // member: integrity
  {
    out << "integrity: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity, out);
    out << ", ";
  }

  // member: test_mode
  {
    out << "test_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.test_mode, out);
    out << ", ";
  }

  // member: bad
  {
    out << "bad: ";
    rosidl_generator_traits::value_to_yaml(msg.bad, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SBASService & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ranging
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ranging: ";
    rosidl_generator_traits::value_to_yaml(msg.ranging, out);
    out << "\n";
  }

  // member: corrections
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "corrections: ";
    rosidl_generator_traits::value_to_yaml(msg.corrections, out);
    out << "\n";
  }

  // member: integrity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integrity: ";
    rosidl_generator_traits::value_to_yaml(msg.integrity, out);
    out << "\n";
  }

  // member: test_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "test_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.test_mode, out);
    out << "\n";
  }

  // member: bad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bad: ";
    rosidl_generator_traits::value_to_yaml(msg.bad, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SBASService & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::SBASService & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::SBASService & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::SBASService>()
{
  return "ublox_ubx_msgs::msg::SBASService";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::SBASService>()
{
  return "ublox_ubx_msgs/msg/SBASService";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::SBASService>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::SBASService>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::SBASService>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__TRAITS_HPP_
