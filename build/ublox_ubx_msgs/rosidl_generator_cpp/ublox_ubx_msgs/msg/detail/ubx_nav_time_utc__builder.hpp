// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavTimeUTC.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_time_utc__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavTimeUTC_utc_std
{
public:
  explicit Init_UBXNavTimeUTC_utc_std(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC utc_std(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_utc_std_type arg)
  {
    msg_.utc_std = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_valid_utc
{
public:
  explicit Init_UBXNavTimeUTC_valid_utc(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_utc_std valid_utc(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_valid_utc_type arg)
  {
    msg_.valid_utc = std::move(arg);
    return Init_UBXNavTimeUTC_utc_std(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_valid_wkn
{
public:
  explicit Init_UBXNavTimeUTC_valid_wkn(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_valid_utc valid_wkn(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_valid_wkn_type arg)
  {
    msg_.valid_wkn = std::move(arg);
    return Init_UBXNavTimeUTC_valid_utc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_valid_tow
{
public:
  explicit Init_UBXNavTimeUTC_valid_tow(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_valid_wkn valid_tow(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_valid_tow_type arg)
  {
    msg_.valid_tow = std::move(arg);
    return Init_UBXNavTimeUTC_valid_wkn(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_sec
{
public:
  explicit Init_UBXNavTimeUTC_sec(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_valid_tow sec(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_sec_type arg)
  {
    msg_.sec = std::move(arg);
    return Init_UBXNavTimeUTC_valid_tow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_min
{
public:
  explicit Init_UBXNavTimeUTC_min(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_sec min(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_min_type arg)
  {
    msg_.min = std::move(arg);
    return Init_UBXNavTimeUTC_sec(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_hour
{
public:
  explicit Init_UBXNavTimeUTC_hour(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_min hour(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_hour_type arg)
  {
    msg_.hour = std::move(arg);
    return Init_UBXNavTimeUTC_min(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_day
{
public:
  explicit Init_UBXNavTimeUTC_day(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_hour day(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_day_type arg)
  {
    msg_.day = std::move(arg);
    return Init_UBXNavTimeUTC_hour(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_month
{
public:
  explicit Init_UBXNavTimeUTC_month(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_day month(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_month_type arg)
  {
    msg_.month = std::move(arg);
    return Init_UBXNavTimeUTC_day(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_year
{
public:
  explicit Init_UBXNavTimeUTC_year(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_month year(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_year_type arg)
  {
    msg_.year = std::move(arg);
    return Init_UBXNavTimeUTC_month(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_nano
{
public:
  explicit Init_UBXNavTimeUTC_nano(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_year nano(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_nano_type arg)
  {
    msg_.nano = std::move(arg);
    return Init_UBXNavTimeUTC_year(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_t_acc
{
public:
  explicit Init_UBXNavTimeUTC_t_acc(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_nano t_acc(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_t_acc_type arg)
  {
    msg_.t_acc = std::move(arg);
    return Init_UBXNavTimeUTC_nano(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_itow
{
public:
  explicit Init_UBXNavTimeUTC_itow(::ublox_ubx_msgs::msg::UBXNavTimeUTC & msg)
  : msg_(msg)
  {}
  Init_UBXNavTimeUTC_t_acc itow(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavTimeUTC_t_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

class Init_UBXNavTimeUTC_header
{
public:
  Init_UBXNavTimeUTC_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavTimeUTC_itow header(::ublox_ubx_msgs::msg::UBXNavTimeUTC::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavTimeUTC_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavTimeUTC msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavTimeUTC>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavTimeUTC_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__BUILDER_HPP_
