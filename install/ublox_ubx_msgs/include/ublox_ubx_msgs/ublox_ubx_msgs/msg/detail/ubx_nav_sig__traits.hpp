// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavSig.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SIG__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_sig__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'sig_data'
#include "ublox_ubx_msgs/msg/detail/sig_data__traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UBXNavSig & msg,
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

  // member: num_sigs
  {
    out << "num_sigs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sigs, out);
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

  // member: sig_data
  {
    if (msg.sig_data.size() == 0) {
      out << "sig_data: []";
    } else {
      out << "sig_data: [";
      size_t pending_items = msg.sig_data.size();
      for (auto item : msg.sig_data) {
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
  const UBXNavSig & msg,
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

  // member: num_sigs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sigs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sigs, out);
    out << "\n";
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

  // member: sig_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sig_data.size() == 0) {
      out << "sig_data: []\n";
    } else {
      out << "sig_data:\n";
      for (auto item : msg.sig_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UBXNavSig & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::UBXNavSig & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::UBXNavSig & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::UBXNavSig>()
{
  return "ublox_ubx_msgs::msg::UBXNavSig";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::UBXNavSig>()
{
  return "ublox_ubx_msgs/msg/UBXNavSig";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::UBXNavSig>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::UBXNavSig>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ublox_ubx_msgs::msg::UBXNavSig>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SIG__TRAITS_HPP_
