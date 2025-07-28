// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavEOE.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_EOE__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_EOE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_eoe__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavEOE_itow
{
public:
  explicit Init_UBXNavEOE_itow(::ublox_ubx_msgs::msg::UBXNavEOE & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavEOE itow(::ublox_ubx_msgs::msg::UBXNavEOE::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavEOE msg_;
};

class Init_UBXNavEOE_header
{
public:
  Init_UBXNavEOE_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavEOE_itow header(::ublox_ubx_msgs::msg::UBXNavEOE::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavEOE_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavEOE msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavEOE>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavEOE_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_EOE__BUILDER_HPP_
