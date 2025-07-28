// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/RawxData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/rawx_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'trk_stat'
#include "ublox_ubx_msgs/msg/detail/trk_stat__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RawxData & msg,
  std::ostream & out)
{
  out << "{";
  // member: pr_mes
  {
    out << "pr_mes: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_mes, out);
    out << ", ";
  }

  // member: cp_mes
  {
    out << "cp_mes: ";
    rosidl_generator_traits::value_to_yaml(msg.cp_mes, out);
    out << ", ";
  }

  // member: do_mes
  {
    out << "do_mes: ";
    rosidl_generator_traits::value_to_yaml(msg.do_mes, out);
    out << ", ";
  }

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

  // member: sig_id
  {
    out << "sig_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sig_id, out);
    out << ", ";
  }

  // member: freq_id
  {
    out << "freq_id: ";
    rosidl_generator_traits::value_to_yaml(msg.freq_id, out);
    out << ", ";
  }

  // member: locktime
  {
    out << "locktime: ";
    rosidl_generator_traits::value_to_yaml(msg.locktime, out);
    out << ", ";
  }

  // member: c_no
  {
    out << "c_no: ";
    rosidl_generator_traits::value_to_yaml(msg.c_no, out);
    out << ", ";
  }

  // member: pr_stdev
  {
    out << "pr_stdev: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_stdev, out);
    out << ", ";
  }

  // member: cp_stdev
  {
    out << "cp_stdev: ";
    rosidl_generator_traits::value_to_yaml(msg.cp_stdev, out);
    out << ", ";
  }

  // member: do_stdev
  {
    out << "do_stdev: ";
    rosidl_generator_traits::value_to_yaml(msg.do_stdev, out);
    out << ", ";
  }

  // member: trk_stat
  {
    out << "trk_stat: ";
    to_flow_style_yaml(msg.trk_stat, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RawxData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pr_mes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pr_mes: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_mes, out);
    out << "\n";
  }

  // member: cp_mes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cp_mes: ";
    rosidl_generator_traits::value_to_yaml(msg.cp_mes, out);
    out << "\n";
  }

  // member: do_mes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "do_mes: ";
    rosidl_generator_traits::value_to_yaml(msg.do_mes, out);
    out << "\n";
  }

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

  // member: sig_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sig_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sig_id, out);
    out << "\n";
  }

  // member: freq_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "freq_id: ";
    rosidl_generator_traits::value_to_yaml(msg.freq_id, out);
    out << "\n";
  }

  // member: locktime
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "locktime: ";
    rosidl_generator_traits::value_to_yaml(msg.locktime, out);
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

  // member: pr_stdev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pr_stdev: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_stdev, out);
    out << "\n";
  }

  // member: cp_stdev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cp_stdev: ";
    rosidl_generator_traits::value_to_yaml(msg.cp_stdev, out);
    out << "\n";
  }

  // member: do_stdev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "do_stdev: ";
    rosidl_generator_traits::value_to_yaml(msg.do_stdev, out);
    out << "\n";
  }

  // member: trk_stat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trk_stat:\n";
    to_block_style_yaml(msg.trk_stat, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RawxData & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::RawxData & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::RawxData & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::RawxData>()
{
  return "ublox_ubx_msgs::msg::RawxData";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::RawxData>()
{
  return "ublox_ubx_msgs/msg/RawxData";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::RawxData>
  : std::integral_constant<bool, has_fixed_size<ublox_ubx_msgs::msg::TrkStat>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::RawxData>
  : std::integral_constant<bool, has_bounded_size<ublox_ubx_msgs::msg::TrkStat>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::RawxData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__TRAITS_HPP_
