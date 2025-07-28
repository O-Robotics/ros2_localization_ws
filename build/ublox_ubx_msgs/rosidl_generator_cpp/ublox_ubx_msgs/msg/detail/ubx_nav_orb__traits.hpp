// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavOrb.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_orb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'sv_info'
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavOrb & msg,
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

  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: num_sv
  {
    out << "num_sv: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sv, out);
    out << ", ";
  }

  // member: reserved_0
  {
    if (msg.reserved_0.size() == 0) {
      out << "reserved_0: []";
    } else {
      out << "reserved_0: [";
      size_t pending_items = msg.reserved_0.size();
      for (auto item : msg.reserved_0) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: sv_info
  {
    if (msg.sv_info.size() == 0) {
      out << "sv_info: []";
    } else {
      out << "sv_info: [";
      size_t pending_items = msg.sv_info.size();
      for (auto item : msg.sv_info) {
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
  const UBXNavOrb & msg,
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

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: num_sv
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sv: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sv, out);
    out << "\n";
  }

  // member: reserved_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.reserved_0.size() == 0) {
      out << "reserved_0: []\n";
    } else {
      out << "reserved_0:\n";
      for (auto item : msg.reserved_0) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: sv_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sv_info.size() == 0) {
      out << "sv_info: []\n";
    } else {
      out << "sv_info:\n";
      for (auto item : msg.sv_info) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavOrb & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavOrb & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavOrb & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavOrb>()
{
  return "ublox_ubx_msgs::msg::UBXNavOrb";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavOrb>()
{
  return "ublox_ubx_msgs/msg/UBXNavOrb";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavOrb>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavOrb>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavOrb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__TRAITS_HPP_
