// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavOrb.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_orb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavOrb_sv_info
{
public:
  explicit Init_UBXNavOrb_sv_info(::ublox_ubx_msgs::msg::UBXNavOrb & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavOrb sv_info(::ublox_ubx_msgs::msg::UBXNavOrb::_sv_info_type arg)
  {
    msg_.sv_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOrb msg_;
};

class Init_UBXNavOrb_reserved_0
{
public:
  explicit Init_UBXNavOrb_reserved_0(::ublox_ubx_msgs::msg::UBXNavOrb & msg)
  : msg_(msg)
  {}
  Init_UBXNavOrb_sv_info reserved_0(::ublox_ubx_msgs::msg::UBXNavOrb::_reserved_0_type arg)
  {
    msg_.reserved_0 = std::move(arg);
    return Init_UBXNavOrb_sv_info(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOrb msg_;
};

class Init_UBXNavOrb_num_sv
{
public:
  explicit Init_UBXNavOrb_num_sv(::ublox_ubx_msgs::msg::UBXNavOrb & msg)
  : msg_(msg)
  {}
  Init_UBXNavOrb_reserved_0 num_sv(::ublox_ubx_msgs::msg::UBXNavOrb::_num_sv_type arg)
  {
    msg_.num_sv = std::move(arg);
    return Init_UBXNavOrb_reserved_0(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOrb msg_;
};

class Init_UBXNavOrb_version
{
public:
  explicit Init_UBXNavOrb_version(::ublox_ubx_msgs::msg::UBXNavOrb & msg)
  : msg_(msg)
  {}
  Init_UBXNavOrb_num_sv version(::ublox_ubx_msgs::msg::UBXNavOrb::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXNavOrb_num_sv(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOrb msg_;
};

class Init_UBXNavOrb_itow
{
public:
  explicit Init_UBXNavOrb_itow(::ublox_ubx_msgs::msg::UBXNavOrb & msg)
  : msg_(msg)
  {}
  Init_UBXNavOrb_version itow(::ublox_ubx_msgs::msg::UBXNavOrb::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavOrb_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOrb msg_;
};

class Init_UBXNavOrb_header
{
public:
  Init_UBXNavOrb_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavOrb_itow header(::ublox_ubx_msgs::msg::UBXNavOrb::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavOrb_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOrb msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavOrb>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavOrb_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__BUILDER_HPP_
