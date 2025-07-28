// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavSBAS.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_sbas__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'service'
#include "ublox_ubx_msgs/msg/detail/sbas_service__traits.hpp"
// Member 'status_flags'
#include "ublox_ubx_msgs/msg/detail/sbas_status_flags__traits.hpp"
// Member 'sv_data'
#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavSBAS & msg,
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

  // member: geo
  {
    out << "geo: ";
    rosidl_generator_traits::value_to_yaml(msg.geo, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: sys
  {
    out << "sys: ";
    rosidl_generator_traits::value_to_yaml(msg.sys, out);
    out << ", ";
  }

  // member: service
  {
    out << "service: ";
    to_flow_style_yaml(msg.service, out);
    out << ", ";
  }

  // member: cnt
  {
    out << "cnt: ";
    rosidl_generator_traits::value_to_yaml(msg.cnt, out);
    out << ", ";
  }

  // member: status_flags
  {
    out << "status_flags: ";
    to_flow_style_yaml(msg.status_flags, out);
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

  // member: sv_data
  {
    if (msg.sv_data.size() == 0) {
      out << "sv_data: []";
    } else {
      out << "sv_data: [";
      size_t pending_items = msg.sv_data.size();
      for (auto item : msg.sv_data) {
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
  const UBXNavSBAS & msg,
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

  // member: geo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geo: ";
    rosidl_generator_traits::value_to_yaml(msg.geo, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: sys
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sys: ";
    rosidl_generator_traits::value_to_yaml(msg.sys, out);
    out << "\n";
  }

  // member: service
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "service:\n";
    to_block_style_yaml(msg.service, out, indentation + 2);
  }

  // member: cnt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cnt: ";
    rosidl_generator_traits::value_to_yaml(msg.cnt, out);
    out << "\n";
  }

  // member: status_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_flags:\n";
    to_block_style_yaml(msg.status_flags, out, indentation + 2);
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

  // member: sv_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sv_data.size() == 0) {
      out << "sv_data: []\n";
    } else {
      out << "sv_data:\n";
      for (auto item : msg.sv_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavSBAS & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavSBAS & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavSBAS & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavSBAS>()
{
  return "ublox_ubx_msgs::msg::UBXNavSBAS";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavSBAS>()
{
  return "ublox_ubx_msgs/msg/UBXNavSBAS";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavSBAS>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavSBAS>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavSBAS>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__TRAITS_HPP_
