// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/SatInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/sat_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'flags'
#include "ublox_ubx_msgs/msg/detail/sat_flags__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SatInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: gnss_id
  {
    out << "gnss_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_id, out);
    out << ", ";
  }

  // member: sv_id
  {
    out << "sv_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_id, out);
    out << ", ";
  }

  // member: cno
  {
    out << "cno: ";
    rosidl_generator_traits::value_to_yaml(msg.cno, out);
    out << ", ";
  }

  // member: elev
  {
    out << "elev: ";
    rosidl_generator_traits::value_to_yaml(msg.elev, out);
    out << ", ";
  }

  // member: azim
  {
    out << "azim: ";
    rosidl_generator_traits::value_to_yaml(msg.azim, out);
    out << ", ";
  }

  // member: pr_res
  {
    out << "pr_res: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_res, out);
    out << ", ";
  }

  // member: flags
  {
    out << "flags: ";
    to_flow_style_yaml(msg.flags, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SatInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: gnss_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gnss_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_id, out);
    out << "\n";
  }

  // member: sv_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sv_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_id, out);
    out << "\n";
  }

  // member: cno
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cno: ";
    rosidl_generator_traits::value_to_yaml(msg.cno, out);
    out << "\n";
  }

  // member: elev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elev: ";
    rosidl_generator_traits::value_to_yaml(msg.elev, out);
    out << "\n";
  }

  // member: azim
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "azim: ";
    rosidl_generator_traits::value_to_yaml(msg.azim, out);
    out << "\n";
  }

  // member: pr_res
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pr_res: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_res, out);
    out << "\n";
  }

  // member: flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flags:\n";
    to_block_style_yaml(msg.flags, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SatInfo & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::SatInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::SatInfo & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::SatInfo>()
{
  return "ublox_ubx_msgs::msg::SatInfo";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::SatInfo>()
{
  return "ublox_ubx_msgs/msg/SatInfo";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::SatInfo>
  : std::integral_constant<bool, has_fixed_size<ublox_ubx_msgs::msg::SatFlags>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::SatInfo>
  : std::integral_constant<bool, has_bounded_size<ublox_ubx_msgs::msg::SatFlags>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::SatInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__TRAITS_HPP_
