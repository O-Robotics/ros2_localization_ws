// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/MeasxData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/measx_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MeasxData & msg,
  std::ostream & out)
{
  out << "{";
  // member: gnss_id
  {
    out << "gnss_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_id, out);
    out << ", ";
  }

  // member: sv_id
  {
    out << "sv_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_id, out);
    out << ", ";
  }

  // member: c_no
  {
    out << "c_no: ";
    rosidl_generator_traits::value_to_yaml(msg.c_no, out);
    out << ", ";
  }

  // member: mpath_indic
  {
    out << "mpath_indic: ";
    rosidl_generator_traits::value_to_yaml(msg.mpath_indic, out);
    out << ", ";
  }

  // member: doppler_ms
  {
    out << "doppler_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.doppler_ms, out);
    out << ", ";
  }

  // member: doppler_hz
  {
    out << "doppler_hz: ";
    rosidl_generator_traits::value_to_yaml(msg.doppler_hz, out);
    out << ", ";
  }

  // member: whole_chips
  {
    out << "whole_chips: ";
    rosidl_generator_traits::value_to_yaml(msg.whole_chips, out);
    out << ", ";
  }

  // member: frac_chips
  {
    out << "frac_chips: ";
    rosidl_generator_traits::value_to_yaml(msg.frac_chips, out);
    out << ", ";
  }

  // member: code_phase
  {
    out << "code_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.code_phase, out);
    out << ", ";
  }

  // member: int_code_phase
  {
    out << "int_code_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.int_code_phase, out);
    out << ", ";
  }

  // member: pseu_range_rms_err
  {
    out << "pseu_range_rms_err: ";
    rosidl_generator_traits::value_to_yaml(msg.pseu_range_rms_err, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeasxData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: gnss_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gnss_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_id, out);
    out << "\n";
  }

  // member: sv_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sv_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_id, out);
    out << "\n";
  }

  // member: c_no
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c_no: ";
    rosidl_generator_traits::value_to_yaml(msg.c_no, out);
    out << "\n";
  }

  // member: mpath_indic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mpath_indic: ";
    rosidl_generator_traits::value_to_yaml(msg.mpath_indic, out);
    out << "\n";
  }

  // member: doppler_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "doppler_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.doppler_ms, out);
    out << "\n";
  }

  // member: doppler_hz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "doppler_hz: ";
    rosidl_generator_traits::value_to_yaml(msg.doppler_hz, out);
    out << "\n";
  }

  // member: whole_chips
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "whole_chips: ";
    rosidl_generator_traits::value_to_yaml(msg.whole_chips, out);
    out << "\n";
  }

  // member: frac_chips
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frac_chips: ";
    rosidl_generator_traits::value_to_yaml(msg.frac_chips, out);
    out << "\n";
  }

  // member: code_phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "code_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.code_phase, out);
    out << "\n";
  }

  // member: int_code_phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "int_code_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.int_code_phase, out);
    out << "\n";
  }

  // member: pseu_range_rms_err
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pseu_range_rms_err: ";
    rosidl_generator_traits::value_to_yaml(msg.pseu_range_rms_err, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasxData & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::MeasxData & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::MeasxData & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::MeasxData>()
{
  return "ublox_ubx_msgs::msg::MeasxData";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::MeasxData>()
{
  return "ublox_ubx_msgs/msg/MeasxData";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::MeasxData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::MeasxData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::MeasxData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__TRAITS_HPP_
