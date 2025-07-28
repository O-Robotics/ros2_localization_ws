// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_msgs:msg/ESFMeasDataItem.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__TRAITS_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_msgs/msg/detail/esf_meas_data_item__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ESFMeasDataItem & msg,
  std::ostream & out)
{
  out << "{";
  // member: data_field
  {
    out << "data_field: ";
    rosidl_generator_traits::value_to_yaml(msg.data_field, out);
    out << ", ";
  }

  // member: data_type
  {
    out << "data_type: ";
    rosidl_generator_traits::value_to_yaml(msg.data_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ESFMeasDataItem & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data_field
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_field: ";
    rosidl_generator_traits::value_to_yaml(msg.data_field, out);
    out << "\n";
  }

  // member: data_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_type: ";
    rosidl_generator_traits::value_to_yaml(msg.data_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ESFMeasDataItem & msg, bool use_flow_style = false)
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
  const ublox_ubx_msgs::msg::ESFMeasDataItem & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_msgs::msg::ESFMeasDataItem & msg)
{
  return ublox_ubx_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_msgs::msg::ESFMeasDataItem>()
{
  return "ublox_ubx_msgs::msg::ESFMeasDataItem";
}

template<>
inline const char * name<ublox_ubx_msgs::msg::ESFMeasDataItem>()
{
  return "ublox_ubx_msgs/msg/ESFMeasDataItem";
}

template<>
struct has_fixed_size<ublox_ubx_msgs::msg::ESFMeasDataItem>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_msgs::msg::ESFMeasDataItem>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_msgs::msg::ESFMeasDataItem>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__TRAITS_HPP_
