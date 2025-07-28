// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavSBAS.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_sbas__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavSBAS_sv_data
{
public:
  explicit Init_UBXNavSBAS_sv_data(::ublox_ubx_msgs::msg::UBXNavSBAS & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavSBAS sv_data(::ublox_ubx_msgs::msg::UBXNavSBAS::_sv_data_type arg)
  {
    msg_.sv_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

class Init_UBXNavSBAS_reserved_0
{
public:
  explicit Init_UBXNavSBAS_reserved_0(::ublox_ubx_msgs::msg::UBXNavSBAS & msg)
  : msg_(msg)
  {}
  Init_UBXNavSBAS_sv_data reserved_0(::ublox_ubx_msgs::msg::UBXNavSBAS::_reserved_0_type arg)
  {
    msg_.reserved_0 = std::move(arg);
    return Init_UBXNavSBAS_sv_data(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

class Init_UBXNavSBAS_status_flags
{
public:
  explicit Init_UBXNavSBAS_status_flags(::ublox_ubx_msgs::msg::UBXNavSBAS & msg)
  : msg_(msg)
  {}
  Init_UBXNavSBAS_reserved_0 status_flags(::ublox_ubx_msgs::msg::UBXNavSBAS::_status_flags_type arg)
  {
    msg_.status_flags = std::move(arg);
    return Init_UBXNavSBAS_reserved_0(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

class Init_UBXNavSBAS_cnt
{
public:
  explicit Init_UBXNavSBAS_cnt(::ublox_ubx_msgs::msg::UBXNavSBAS & msg)
  : msg_(msg)
  {}
  Init_UBXNavSBAS_status_flags cnt(::ublox_ubx_msgs::msg::UBXNavSBAS::_cnt_type arg)
  {
    msg_.cnt = std::move(arg);
    return Init_UBXNavSBAS_status_flags(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

class Init_UBXNavSBAS_service
{
public:
  explicit Init_UBXNavSBAS_service(::ublox_ubx_msgs::msg::UBXNavSBAS & msg)
  : msg_(msg)
  {}
  Init_UBXNavSBAS_cnt service(::ublox_ubx_msgs::msg::UBXNavSBAS::_service_type arg)
  {
    msg_.service = std::move(arg);
    return Init_UBXNavSBAS_cnt(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

class Init_UBXNavSBAS_sys
{
public:
  explicit Init_UBXNavSBAS_sys(::ublox_ubx_msgs::msg::UBXNavSBAS & msg)
  : msg_(msg)
  {}
  Init_UBXNavSBAS_service sys(::ublox_ubx_msgs::msg::UBXNavSBAS::_sys_type arg)
  {
    msg_.sys = std::move(arg);
    return Init_UBXNavSBAS_service(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

class Init_UBXNavSBAS_mode
{
public:
  explicit Init_UBXNavSBAS_mode(::ublox_ubx_msgs::msg::UBXNavSBAS & msg)
  : msg_(msg)
  {}
  Init_UBXNavSBAS_sys mode(::ublox_ubx_msgs::msg::UBXNavSBAS::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_UBXNavSBAS_sys(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

class Init_UBXNavSBAS_geo
{
public:
  explicit Init_UBXNavSBAS_geo(::ublox_ubx_msgs::msg::UBXNavSBAS & msg)
  : msg_(msg)
  {}
  Init_UBXNavSBAS_mode geo(::ublox_ubx_msgs::msg::UBXNavSBAS::_geo_type arg)
  {
    msg_.geo = std::move(arg);
    return Init_UBXNavSBAS_mode(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

class Init_UBXNavSBAS_itow
{
public:
  explicit Init_UBXNavSBAS_itow(::ublox_ubx_msgs::msg::UBXNavSBAS & msg)
  : msg_(msg)
  {}
  Init_UBXNavSBAS_geo itow(::ublox_ubx_msgs::msg::UBXNavSBAS::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavSBAS_geo(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

class Init_UBXNavSBAS_header
{
public:
  Init_UBXNavSBAS_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavSBAS_itow header(::ublox_ubx_msgs::msg::UBXNavSBAS::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavSBAS_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSBAS msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavSBAS>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavSBAS_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__BUILDER_HPP_
