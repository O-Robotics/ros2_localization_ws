// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmRawx.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'rec_stat'
#include "ublox_ubx_msgs/msg/detail/rec_stat__traits.hpp"
// Member 'rawx_data'
#include "ublox_ubx_msgs/msg/detail/rawx_data__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXRxmRawx & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: rcv_tow
  {
    out << "rcv_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.rcv_tow, out);
    out << ", ";
  }

  // member: week
  {
    out << "week: ";
    rosidl_generator_traits::value_to_yaml(msg.week, out);
    out << ", ";
  }

  // member: leap_s
  {
    out << "leap_s: ";
    rosidl_generator_traits::value_to_yaml(msg.leap_s, out);
    out << ", ";
  }

  // member: num_meas
  {
    out << "num_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.num_meas, out);
    out << ", ";
  }

  // member: rec_stat
  {
    out << "rec_stat: ";
    to_flow_style_yaml(msg.rec_stat, out);
    out << ", ";
  }

  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: rawx_data
  {
    if (msg.rawx_data.size() == 0) {
      out << "rawx_data: []";
    } else {
      out << "rawx_data: [";
      size_t pending_items = msg.rawx_data.size();
      for (auto item : msg.rawx_data) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXRxmRawx & msg,
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

  // member: rcv_tow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rcv_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.rcv_tow, out);
    out << "\n";
  }

  // member: week
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "week: ";
    rosidl_generator_traits::value_to_yaml(msg.week, out);
    out << "\n";
  }

  // member: leap_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "leap_s: ";
    rosidl_generator_traits::value_to_yaml(msg.leap_s, out);
    out << "\n";
  }

  // member: num_meas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.num_meas, out);
    out << "\n";
  }

  // member: rec_stat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rec_stat:\n";
    to_block_style_yaml(msg.rec_stat, out, indentation + 2);
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

  // member: rawx_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.rawx_data.size() == 0) {
      out << "rawx_data: []\n";
    } else {
      out << "rawx_data:\n";
      for (auto item : msg.rawx_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXRxmRawx & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXRxmRawx & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXRxmRawx & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXRxmRawx>()
{
  return "ublox_ubx_msgs::msg::UBXRxmRawx";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXRxmRawx>()
{
  return "ublox_ubx_msgs/msg/UBXRxmRawx";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXRxmRawx>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXRxmRawx>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXRxmRawx>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__TRAITS_HPP_
