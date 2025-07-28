// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavTimeUTC.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_time_utc__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'utc_std'
#include "ublox_ubx_msgs/msg/detail/utc_std__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavTimeUTC & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: itow
  {
    out << "itow: ";
    rosidl_generator_traits::value_to_yaml(msg.itow, out);
    out << ", ";
  }

  // member: t_acc
  {
    out << "t_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.t_acc, out);
    out << ", ";
  }

  // member: nano
  {
    out << "nano: ";
    rosidl_generator_traits::value_to_yaml(msg.nano, out);
    out << ", ";
  }

  // member: year
  {
    out << "year: ";
    rosidl_generator_traits::value_to_yaml(msg.year, out);
    out << ", ";
  }

  // member: month
  {
    out << "month: ";
    rosidl_generator_traits::value_to_yaml(msg.month, out);
    out << ", ";
  }

  // member: day
  {
    out << "day: ";
    rosidl_generator_traits::value_to_yaml(msg.day, out);
    out << ", ";
  }

  // member: hour
  {
    out << "hour: ";
    rosidl_generator_traits::value_to_yaml(msg.hour, out);
    out << ", ";
  }

  // member: min
  {
    out << "min: ";
    rosidl_generator_traits::value_to_yaml(msg.min, out);
    out << ", ";
  }

  // member: sec
  {
    out << "sec: ";
    rosidl_generator_traits::value_to_yaml(msg.sec, out);
    out << ", ";
  }

  // member: valid_tow
  {
    out << "valid_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_tow, out);
    out << ", ";
  }

  // member: valid_wkn
  {
    out << "valid_wkn: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_wkn, out);
    out << ", ";
  }

  // member: valid_utc
  {
    out << "valid_utc: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_utc, out);
    out << ", ";
  }

  // member: utc_std
  {
    out << "utc_std: ";
    to_flow_style_yaml(msg.utc_std, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXNavTimeUTC & msg,
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

  // member: itow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "itow: ";
    rosidl_generator_traits::value_to_yaml(msg.itow, out);
    out << "\n";
  }

  // member: t_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.t_acc, out);
    out << "\n";
  }

  // member: nano
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nano: ";
    rosidl_generator_traits::value_to_yaml(msg.nano, out);
    out << "\n";
  }

  // member: year
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "year: ";
    rosidl_generator_traits::value_to_yaml(msg.year, out);
    out << "\n";
  }

  // member: month
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "month: ";
    rosidl_generator_traits::value_to_yaml(msg.month, out);
    out << "\n";
  }

  // member: day
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "day: ";
    rosidl_generator_traits::value_to_yaml(msg.day, out);
    out << "\n";
  }

  // member: hour
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hour: ";
    rosidl_generator_traits::value_to_yaml(msg.hour, out);
    out << "\n";
  }

  // member: min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min: ";
    rosidl_generator_traits::value_to_yaml(msg.min, out);
    out << "\n";
  }

  // member: sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sec: ";
    rosidl_generator_traits::value_to_yaml(msg.sec, out);
    out << "\n";
  }

  // member: valid_tow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_tow, out);
    out << "\n";
  }

  // member: valid_wkn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid_wkn: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_wkn, out);
    out << "\n";
  }

  // member: valid_utc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid_utc: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_utc, out);
    out << "\n";
  }

  // member: utc_std
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "utc_std:\n";
    to_block_style_yaml(msg.utc_std, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavTimeUTC & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavTimeUTC & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavTimeUTC>()
{
  return "ublox_ubx_msgs::msg::UBXNavTimeUTC";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavTimeUTC>()
{
  return "ublox_ubx_msgs/msg/UBXNavTimeUTC";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavTimeUTC>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<ublox_ubx_msgs::msg::UtcStd>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavTimeUTC>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<ublox_ubx_msgs::msg::UtcStd>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavTimeUTC>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__TRAITS_HPP_
