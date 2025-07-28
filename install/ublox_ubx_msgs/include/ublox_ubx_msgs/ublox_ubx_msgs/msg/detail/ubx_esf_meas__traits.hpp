// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXEsfMeas.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_MEAS__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_MEAS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_esf_meas__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'data'
#include "ublox_ubx_msgs/msg/detail/esf_meas_data_item__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXEsfMeas & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: time_tag
  {
    out << "time_tag: ";
    rosidl_generator_traits::value_to_yaml(msg.time_tag, out);
    out << ", ";
  }

  // member: time_mark_sent
  {
    out << "time_mark_sent: ";
    rosidl_generator_traits::value_to_yaml(msg.time_mark_sent, out);
    out << ", ";
  }

  // member: time_mark_edge
  {
    out << "time_mark_edge: ";
    rosidl_generator_traits::value_to_yaml(msg.time_mark_edge, out);
    out << ", ";
  }

  // member: calib_ttag_valid
  {
    out << "calib_ttag_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.calib_ttag_valid, out);
    out << ", ";
  }

  // member: num_meas
  {
    out << "num_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.num_meas, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: calib_ttag
  {
    out << "calib_ttag: ";
    rosidl_generator_traits::value_to_yaml(msg.calib_ttag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UBXEsfMeas & msg,
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

  // member: time_tag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_tag: ";
    rosidl_generator_traits::value_to_yaml(msg.time_tag, out);
    out << "\n";
  }

  // member: time_mark_sent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_mark_sent: ";
    rosidl_generator_traits::value_to_yaml(msg.time_mark_sent, out);
    out << "\n";
  }

  // member: time_mark_edge
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_mark_edge: ";
    rosidl_generator_traits::value_to_yaml(msg.time_mark_edge, out);
    out << "\n";
  }

  // member: calib_ttag_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calib_ttag_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.calib_ttag_valid, out);
    out << "\n";
  }

  // member: num_meas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_meas: ";
    rosidl_generator_traits::value_to_yaml(msg.num_meas, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: calib_ttag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calib_ttag: ";
    rosidl_generator_traits::value_to_yaml(msg.calib_ttag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXEsfMeas & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXEsfMeas & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXEsfMeas & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXEsfMeas>()
{
  return "ublox_ubx_msgs::msg::UBXEsfMeas";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXEsfMeas>()
{
  return "ublox_ubx_msgs/msg/UBXEsfMeas";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXEsfMeas>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXEsfMeas>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXEsfMeas>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_MEAS__TRAITS_HPP_
