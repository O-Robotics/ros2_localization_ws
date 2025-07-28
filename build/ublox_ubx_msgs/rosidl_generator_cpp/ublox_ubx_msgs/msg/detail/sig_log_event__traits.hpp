// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/SigLogEvent.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/sig_log_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SigLogEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: time_elapsed
  {
    out << "time_elapsed: ";
    rosidl_generator_traits::value_to_yaml(msg.time_elapsed, out);
    out << ", ";
  }

  // member: detection_type
  {
    out << "detection_type: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_type, out);
    out << ", ";
  }

  // member: event_type
  {
    out << "event_type: ";
    rosidl_generator_traits::value_to_yaml(msg.event_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SigLogEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: time_elapsed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_elapsed: ";
    rosidl_generator_traits::value_to_yaml(msg.time_elapsed, out);
    out << "\n";
  }

  // member: detection_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection_type: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_type, out);
    out << "\n";
  }

  // member: event_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "event_type: ";
    rosidl_generator_traits::value_to_yaml(msg.event_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SigLogEvent & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::SigLogEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::SigLogEvent & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::SigLogEvent>()
{
  return "ublox_ubx_msgs::msg::SigLogEvent";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::SigLogEvent>()
{
  return "ublox_ubx_msgs/msg/SigLogEvent";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::SigLogEvent>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::SigLogEvent>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::SigLogEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__TRAITS_HPP_
