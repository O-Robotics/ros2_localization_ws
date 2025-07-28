// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/SigFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/sig_flags__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SigFlags & msg,
  std::ostream & out)
{
  out << "{";
  // member: health
  {
    out << "health: ";
    rosidl_generator_traits::value_to_yaml(msg.health, out);
    out << ", ";
  }

  // member: pr_smoothed
  {
    out << "pr_smoothed: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_smoothed, out);
    out << ", ";
  }

  // member: pr_used
  {
    out << "pr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_used, out);
    out << ", ";
  }

  // member: cr_used
  {
    out << "cr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.cr_used, out);
    out << ", ";
  }

  // member: do_used
  {
    out << "do_used: ";
    rosidl_generator_traits::value_to_yaml(msg.do_used, out);
    out << ", ";
  }

  // member: pr_corr_used
  {
    out << "pr_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_corr_used, out);
    out << ", ";
  }

  // member: cr_corr_used
  {
    out << "cr_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.cr_corr_used, out);
    out << ", ";
  }

  // member: do_corr_used
  {
    out << "do_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.do_corr_used, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SigFlags & msg,
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

  // member: pr_smoothed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pr_smoothed: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_smoothed, out);
    out << "\n";
  }

  // member: pr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_used, out);
    out << "\n";
  }

  // member: cr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.cr_used, out);
    out << "\n";
  }

  // member: do_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "do_used: ";
    rosidl_generator_traits::value_to_yaml(msg.do_used, out);
    out << "\n";
  }

  // member: pr_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pr_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_corr_used, out);
    out << "\n";
  }

  // member: cr_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cr_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.cr_corr_used, out);
    out << "\n";
  }

  // member: do_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "do_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.do_corr_used, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SigFlags & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::SigFlags & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::SigFlags & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::SigFlags>()
{
  return "ublox_ubx_msgs::msg::SigFlags";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::SigFlags>()
{
  return "ublox_ubx_msgs/msg/SigFlags";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::SigFlags>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::SigFlags>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::SigFlags>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__TRAITS_HPP_
