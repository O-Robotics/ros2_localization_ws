// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/ESFSensorStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ESFSensorStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: sensor_data_type
  {
    out << "sensor_data_type: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_data_type, out);
    out << ", ";
  }

  // member: used
  {
    out << "used: ";
    rosidl_generator_traits::value_to_yaml(msg.used, out);
    out << ", ";
  }

  // member: ready
  {
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
    out << ", ";
  }

  // member: calib_status
  {
    out << "calib_status: ";
    rosidl_generator_traits::value_to_yaml(msg.calib_status, out);
    out << ", ";
  }

  // member: time_status
  {
    out << "time_status: ";
    rosidl_generator_traits::value_to_yaml(msg.time_status, out);
    out << ", ";
  }

  // member: freq
  {
    out << "freq: ";
    rosidl_generator_traits::value_to_yaml(msg.freq, out);
    out << ", ";
  }

  // member: fault_bad_meas
  {
    out << "fault_bad_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_bad_meas, out);
    out << ", ";
  }

  // member: fault_bad_ttag
  {
    out << "fault_bad_ttag: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_bad_ttag, out);
    out << ", ";
  }

  // member: fault_missing_meas
  {
    out << "fault_missing_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_missing_meas, out);
    out << ", ";
  }

  // member: fault_noisy_meas
  {
    out << "fault_noisy_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_noisy_meas, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ESFSensorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sensor_data_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_data_type: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor_data_type, out);
    out << "\n";
  }

  // member: used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "used: ";
    rosidl_generator_traits::value_to_yaml(msg.used, out);
    out << "\n";
  }

  // member: ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
    out << "\n";
  }

  // member: calib_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calib_status: ";
    rosidl_generator_traits::value_to_yaml(msg.calib_status, out);
    out << "\n";
  }

  // member: time_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_status: ";
    rosidl_generator_traits::value_to_yaml(msg.time_status, out);
    out << "\n";
  }

  // member: freq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "freq: ";
    rosidl_generator_traits::value_to_yaml(msg.freq, out);
    out << "\n";
  }

  // member: fault_bad_meas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_bad_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_bad_meas, out);
    out << "\n";
  }

  // member: fault_bad_ttag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_bad_ttag: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_bad_ttag, out);
    out << "\n";
  }

  // member: fault_missing_meas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_missing_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_missing_meas, out);
    out << "\n";
  }

  // member: fault_noisy_meas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_noisy_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_noisy_meas, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ESFSensorStatus & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::ESFSensorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::ESFSensorStatus & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::ESFSensorStatus>()
{
  return "ublox_ubx_msgs::msg::ESFSensorStatus";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::ESFSensorStatus>()
{
  return "ublox_ubx_msgs/msg/ESFSensorStatus";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::ESFSensorStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::ESFSensorStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::ESFSensorStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__TRAITS_HPP_
