// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXEsfStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_esf_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'sensor_statuses'
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXEsfStatus & msg,
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

  // member: wt_init_status
  {
    out << "wt_init_status: ";
    rosidl_generator_traits::value_to_yaml(msg.wt_init_status, out);
    out << ", ";
  }

  // member: mnt_alg_status
  {
    out << "mnt_alg_status: ";
    rosidl_generator_traits::value_to_yaml(msg.mnt_alg_status, out);
    out << ", ";
  }

  // member: ins_init_status
  {
    out << "ins_init_status: ";
    rosidl_generator_traits::value_to_yaml(msg.ins_init_status, out);
    out << ", ";
  }

  // member: imu_init_status
  {
    out << "imu_init_status: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_init_status, out);
    out << ", ";
  }

  // member: fusion_mode
  {
    out << "fusion_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_mode, out);
    out << ", ";
  }

  // member: num_sens
  {
    out << "num_sens: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sens, out);
    out << ", ";
  }

  // member: sensor_statuses
  {
    if (msg.sensor_statuses.size() == 0) {
      out << "sensor_statuses: []";
    } else {
      out << "sensor_statuses: [";
      size_t pending_items = msg.sensor_statuses.size();
      for (auto item : msg.sensor_statuses) {
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
  const UBXEsfStatus & msg,
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

  // member: wt_init_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wt_init_status: ";
    rosidl_generator_traits::value_to_yaml(msg.wt_init_status, out);
    out << "\n";
  }

  // member: mnt_alg_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mnt_alg_status: ";
    rosidl_generator_traits::value_to_yaml(msg.mnt_alg_status, out);
    out << "\n";
  }

  // member: ins_init_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ins_init_status: ";
    rosidl_generator_traits::value_to_yaml(msg.ins_init_status, out);
    out << "\n";
  }

  // member: imu_init_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_init_status: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_init_status, out);
    out << "\n";
  }

  // member: fusion_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fusion_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.fusion_mode, out);
    out << "\n";
  }

  // member: num_sens
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sens: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sens, out);
    out << "\n";
  }

  // member: sensor_statuses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sensor_statuses.size() == 0) {
      out << "sensor_statuses: []\n";
    } else {
      out << "sensor_statuses:\n";
      for (auto item : msg.sensor_statuses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXEsfStatus & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXEsfStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXEsfStatus & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXEsfStatus>()
{
  return "ublox_ubx_msgs::msg::UBXEsfStatus";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXEsfStatus>()
{
  return "ublox_ubx_msgs/msg/UBXEsfStatus";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXEsfStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXEsfStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXEsfStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__TRAITS_HPP_
