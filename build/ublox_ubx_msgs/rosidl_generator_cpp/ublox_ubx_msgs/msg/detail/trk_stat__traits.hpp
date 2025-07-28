// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/TrkStat.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/trk_stat__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrkStat & msg,
  std::ostream & out)
{
  out << "{";
  // member: pr_valid
  {
    out << "pr_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_valid, out);
    out << ", ";
  }

  // member: cp_valid
  {
    out << "cp_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.cp_valid, out);
    out << ", ";
  }

  // member: half_cyc
  {
    out << "half_cyc: ";
    rosidl_generator_traits::value_to_yaml(msg.half_cyc, out);
    out << ", ";
  }

  // member: sub_half_cyc
  {
    out << "sub_half_cyc: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_half_cyc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrkStat & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pr_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pr_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_valid, out);
    out << "\n";
  }

  // member: cp_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cp_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.cp_valid, out);
    out << "\n";
  }

  // member: half_cyc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "half_cyc: ";
    rosidl_generator_traits::value_to_yaml(msg.half_cyc, out);
    out << "\n";
  }

  // member: sub_half_cyc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sub_half_cyc: ";
    rosidl_generator_traits::value_to_yaml(msg.sub_half_cyc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrkStat & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::TrkStat & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::TrkStat & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::TrkStat>()
{
  return "ublox_ubx_msgs::msg::TrkStat";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::TrkStat>()
{
  return "ublox_ubx_msgs/msg/TrkStat";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::TrkStat>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::TrkStat>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::TrkStat>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__TRAITS_HPP_
