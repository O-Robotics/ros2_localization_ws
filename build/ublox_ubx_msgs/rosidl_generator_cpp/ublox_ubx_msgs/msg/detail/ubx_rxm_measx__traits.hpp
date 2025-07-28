// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmMeasx.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_rxm_measx__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'sv_data'
#include "ublox_ubx_msgs/msg/detail/measx_data__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXRxmMeasx & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: gps_tow
  {
    out << "gps_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_tow, out);
    out << ", ";
  }

  // member: glo_tow
  {
    out << "glo_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.glo_tow, out);
    out << ", ";
  }

  // member: bds_tow
  {
    out << "bds_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.bds_tow, out);
    out << ", ";
  }

  // member: qzss_tow
  {
    out << "qzss_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.qzss_tow, out);
    out << ", ";
  }

  // member: gps_tow_acc
  {
    out << "gps_tow_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_tow_acc, out);
    out << ", ";
  }

  // member: glo_tow_acc
  {
    out << "glo_tow_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.glo_tow_acc, out);
    out << ", ";
  }

  // member: bds_tow_acc
  {
    out << "bds_tow_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.bds_tow_acc, out);
    out << ", ";
  }

  // member: qzss_tow_acc
  {
    out << "qzss_tow_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.qzss_tow_acc, out);
    out << ", ";
  }

  // member: num_sv
  {
    out << "num_sv: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sv, out);
    out << ", ";
  }

  // member: flags
  {
    out << "flags: ";
    rosidl_generator_traits::value_to_yaml(msg.flags, out);
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
  const UBXRxmMeasx & msg,
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

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: gps_tow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_tow, out);
    out << "\n";
  }

  // member: glo_tow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "glo_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.glo_tow, out);
    out << "\n";
  }

  // member: bds_tow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bds_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.bds_tow, out);
    out << "\n";
  }

  // member: qzss_tow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "qzss_tow: ";
    rosidl_generator_traits::value_to_yaml(msg.qzss_tow, out);
    out << "\n";
  }

  // member: gps_tow_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_tow_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_tow_acc, out);
    out << "\n";
  }

  // member: glo_tow_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "glo_tow_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.glo_tow_acc, out);
    out << "\n";
  }

  // member: bds_tow_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bds_tow_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.bds_tow_acc, out);
    out << "\n";
  }

  // member: qzss_tow_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "qzss_tow_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.qzss_tow_acc, out);
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

  // member: flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flags: ";
    rosidl_generator_traits::value_to_yaml(msg.flags, out);
    out << "\n";
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

inline std::string to_yaml(const UBXRxmMeasx & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXRxmMeasx & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXRxmMeasx>()
{
  return "ublox_ubx_msgs::msg::UBXRxmMeasx";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXRxmMeasx>()
{
  return "ublox_ubx_msgs/msg/UBXRxmMeasx";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXRxmMeasx>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXRxmMeasx>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXRxmMeasx>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__TRAITS_HPP_
