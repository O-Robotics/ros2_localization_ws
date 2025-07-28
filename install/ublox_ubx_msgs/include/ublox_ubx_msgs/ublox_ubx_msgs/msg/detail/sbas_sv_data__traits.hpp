// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/SBASSvData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SBASSvData & msg,
  std::ostream & out)
{
  out << "{";
  // member: svid
  {
    out << "svid: ";
    rosidl_generator_traits::value_to_yaml(msg.svid, out);
    out << ", ";
  }

  // member: reserved_1
  {
    out << "reserved_1: ";
    rosidl_generator_traits::value_to_yaml(msg.reserved_1, out);
    out << ", ";
  }

  // member: udre
  {
    out << "udre: ";
    rosidl_generator_traits::value_to_yaml(msg.udre, out);
    out << ", ";
  }

  // member: sv_sys
  {
    out << "sv_sys: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_sys, out);
    out << ", ";
  }

  // member: sv_service
  {
    out << "sv_service: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_service, out);
    out << ", ";
  }

  // member: reserved_2
  {
    out << "reserved_2: ";
    rosidl_generator_traits::value_to_yaml(msg.reserved_2, out);
    out << ", ";
  }

  // member: prc
  {
    out << "prc: ";
    rosidl_generator_traits::value_to_yaml(msg.prc, out);
    out << ", ";
  }

  // member: reserved_3
  {
    if (msg.reserved_3.size() == 0) {
      out << "reserved_3: []";
    } else {
      out << "reserved_3: [";
      size_t pending_items = msg.reserved_3.size();
      for (auto item : msg.reserved_3) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ic
  {
    out << "ic: ";
    rosidl_generator_traits::value_to_yaml(msg.ic, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SBASSvData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: svid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "svid: ";
    rosidl_generator_traits::value_to_yaml(msg.svid, out);
    out << "\n";
  }

  // member: reserved_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved_1: ";
    rosidl_generator_traits::value_to_yaml(msg.reserved_1, out);
    out << "\n";
  }

  // member: udre
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "udre: ";
    rosidl_generator_traits::value_to_yaml(msg.udre, out);
    out << "\n";
  }

  // member: sv_sys
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sv_sys: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_sys, out);
    out << "\n";
  }

  // member: sv_service
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sv_service: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_service, out);
    out << "\n";
  }

  // member: reserved_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved_2: ";
    rosidl_generator_traits::value_to_yaml(msg.reserved_2, out);
    out << "\n";
  }

  // member: prc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prc: ";
    rosidl_generator_traits::value_to_yaml(msg.prc, out);
    out << "\n";
  }

  // member: reserved_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.reserved_3.size() == 0) {
      out << "reserved_3: []\n";
    } else {
      out << "reserved_3:\n";
      for (auto item : msg.reserved_3) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ic: ";
    rosidl_generator_traits::value_to_yaml(msg.ic, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SBASSvData & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::SBASSvData & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::SBASSvData & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::SBASSvData>()
{
  return "ublox_ubx_msgs::msg::SBASSvData";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::SBASSvData>()
{
  return "ublox_ubx_msgs/msg/SBASSvData";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::SBASSvData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::SBASSvData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::SBASSvData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__TRAITS_HPP_
