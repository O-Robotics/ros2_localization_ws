// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/SatFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/sat_flags__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SatFlags & msg,
  std::ostream & out)
{
  out << "{";
  // member: quality_ind
  {
    out << "quality_ind: ";
    rosidl_generator_traits::value_to_yaml(msg.quality_ind, out);
    out << ", ";
  }

  // member: sv_used
  {
    out << "sv_used: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_used, out);
    out << ", ";
  }

  // member: health
  {
    out << "health: ";
    rosidl_generator_traits::value_to_yaml(msg.health, out);
    out << ", ";
  }

  // member: diff_corr
  {
    out << "diff_corr: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_corr, out);
    out << ", ";
  }

  // member: smoothed
  {
    out << "smoothed: ";
    rosidl_generator_traits::value_to_yaml(msg.smoothed, out);
    out << ", ";
  }

  // member: orbit_source
  {
    out << "orbit_source: ";
    rosidl_generator_traits::value_to_yaml(msg.orbit_source, out);
    out << ", ";
  }

  // member: eph_avail
  {
    out << "eph_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.eph_avail, out);
    out << ", ";
  }

  // member: alm_avail
  {
    out << "alm_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.alm_avail, out);
    out << ", ";
  }

  // member: ano_avail
  {
    out << "ano_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.ano_avail, out);
    out << ", ";
  }

  // member: aop_avail
  {
    out << "aop_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.aop_avail, out);
    out << ", ";
  }

  // member: sbas_corr_used
  {
    out << "sbas_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.sbas_corr_used, out);
    out << ", ";
  }

  // member: rtcm_corr_used
  {
    out << "rtcm_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.rtcm_corr_used, out);
    out << ", ";
  }

  // member: slas_corr_used
  {
    out << "slas_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.slas_corr_used, out);
    out << ", ";
  }

  // member: spartn_corr_used
  {
    out << "spartn_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.spartn_corr_used, out);
    out << ", ";
  }

  // member: pr_corr_used
  {
    out << "pr_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_corr_used, out);
    out << ", ";
  }

  // member: cr_corr_used
  {
    out << "cr_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.cr_corr_used, out);
    out << ", ";
  }

  // member: do_corr_used
  {
    out << "do_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.do_corr_used, out);
    out << ", ";
  }

  // member: clas_corr_used
  {
    out << "clas_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.clas_corr_used, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SatFlags & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: quality_ind
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quality_ind: ";
    rosidl_generator_traits::value_to_yaml(msg.quality_ind, out);
    out << "\n";
  }

  // member: sv_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sv_used: ";
    rosidl_generator_traits::value_to_yaml(msg.sv_used, out);
    out << "\n";
  }

  // member: health
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "health: ";
    rosidl_generator_traits::value_to_yaml(msg.health, out);
    out << "\n";
  }

  // member: diff_corr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diff_corr: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_corr, out);
    out << "\n";
  }

  // member: smoothed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "smoothed: ";
    rosidl_generator_traits::value_to_yaml(msg.smoothed, out);
    out << "\n";
  }

  // member: orbit_source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orbit_source: ";
    rosidl_generator_traits::value_to_yaml(msg.orbit_source, out);
    out << "\n";
  }

  // member: eph_avail
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eph_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.eph_avail, out);
    out << "\n";
  }

  // member: alm_avail
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alm_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.alm_avail, out);
    out << "\n";
  }

  // member: ano_avail
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ano_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.ano_avail, out);
    out << "\n";
  }

  // member: aop_avail
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aop_avail: ";
    rosidl_generator_traits::value_to_yaml(msg.aop_avail, out);
    out << "\n";
  }

  // member: sbas_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sbas_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.sbas_corr_used, out);
    out << "\n";
  }

  // member: rtcm_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rtcm_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.rtcm_corr_used, out);
    out << "\n";
  }

  // member: slas_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slas_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.slas_corr_used, out);
    out << "\n";
  }

  // member: spartn_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spartn_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.spartn_corr_used, out);
    out << "\n";
  }

  // member: pr_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pr_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.pr_corr_used, out);
    out << "\n";
  }

  // member: cr_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cr_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.cr_corr_used, out);
    out << "\n";
  }

  // member: do_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "do_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.do_corr_used, out);
    out << "\n";
  }

  // member: clas_corr_used
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "clas_corr_used: ";
    rosidl_generator_traits::value_to_yaml(msg.clas_corr_used, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SatFlags & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::SatFlags & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::SatFlags & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::SatFlags>()
{
  return "ublox_ubx_msgs::msg::SatFlags";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::SatFlags>()
{
  return "ublox_ubx_msgs/msg/SatFlags";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::SatFlags>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::SatFlags>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::SatFlags>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__TRAITS_HPP_
