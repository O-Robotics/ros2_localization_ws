// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/SigData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/sig_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'sig_flags'
#include "ublox_ubx_msgs/msg/detail/sig_flags__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SigData & msg,
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

  // member: pr_res
  {
    out << "pr_res: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_res, out);
    out << ", ";
  }

  // member: cno
  {
    out << "cno: ";
    rosidl_generator_traits::value_to_yaml(msg.cno, out);
    out << ", ";
  }

  // member: quality_ind
  {
    out << "quality_ind: ";
    rosidl_generator_traits::value_to_yaml(msg.quality_ind, out);
    out << ", ";
  }

  // member: corr_source
  {
    out << "corr_source: ";
    rosidl_generator_traits::value_to_yaml(msg.corr_source, out);
    out << ", ";
  }

  // member: iono_model
  {
    out << "iono_model: ";
    rosidl_generator_traits::value_to_yaml(msg.iono_model, out);
    out << ", ";
  }

  // member: sig_flags
  {
    out << "sig_flags: ";
    to_flow_style_yaml(msg.sig_flags, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SigData & msg,
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

  // member: pr_res
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pr_res: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_res, out);
    out << "\n";
  }

  // member: cno
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cno: ";
    rosidl_generator_traits::value_to_yaml(msg.cno, out);
    out << "\n";
  }

  // member: quality_ind
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quality_ind: ";
    rosidl_generator_traits::value_to_yaml(msg.quality_ind, out);
    out << "\n";
  }

  // member: corr_source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "corr_source: ";
    rosidl_generator_traits::value_to_yaml(msg.corr_source, out);
    out << "\n";
  }

  // member: iono_model
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iono_model: ";
    rosidl_generator_traits::value_to_yaml(msg.iono_model, out);
    out << "\n";
  }

  // member: sig_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sig_flags:\n";
    to_block_style_yaml(msg.sig_flags, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SigData & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::SigData & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::SigData & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::SigData>()
{
  return "ublox_ubx_msgs::msg::SigData";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::SigData>()
{
  return "ublox_ubx_msgs/msg/SigData";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::SigData>
  : std::integral_constant<bool, has_fixed_size<ublox_ubx_msgs::msg::SigFlags>::value> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::SigData>
  : std::integral_constant<bool, has_bounded_size<ublox_ubx_msgs::msg::SigFlags>::value> {};

template<>
struct is_message<ublox_ubx_msgs::msg::SigData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__TRAITS_HPP_
