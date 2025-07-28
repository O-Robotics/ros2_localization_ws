// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmRawx.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXRxmRawx_rawx_data
{
public:
  explicit Init_UBXRxmRawx_rawx_data(::ublox_ubx_msgs::msg::UBXRxmRawx & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXRxmRawx rawx_data(::ublox_ubx_msgs::msg::UBXRxmRawx::_rawx_data_type arg)
  {
    msg_.rawx_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRawx msg_;
};

class Init_UBXRxmRawx_version
{
public:
  explicit Init_UBXRxmRawx_version(::ublox_ubx_msgs::msg::UBXRxmRawx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRawx_rawx_data version(::ublox_ubx_msgs::msg::UBXRxmRawx::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXRxmRawx_rawx_data(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRawx msg_;
};

class Init_UBXRxmRawx_rec_stat
{
public:
  explicit Init_UBXRxmRawx_rec_stat(::ublox_ubx_msgs::msg::UBXRxmRawx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRawx_version rec_stat(::ublox_ubx_msgs::msg::UBXRxmRawx::_rec_stat_type arg)
  {
    msg_.rec_stat = std::move(arg);
    return Init_UBXRxmRawx_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRawx msg_;
};

class Init_UBXRxmRawx_num_meas
{
public:
  explicit Init_UBXRxmRawx_num_meas(::ublox_ubx_msgs::msg::UBXRxmRawx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRawx_rec_stat num_meas(::ublox_ubx_msgs::msg::UBXRxmRawx::_num_meas_type arg)
  {
    msg_.num_meas = std::move(arg);
    return Init_UBXRxmRawx_rec_stat(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRawx msg_;
};

class Init_UBXRxmRawx_leap_s
{
public:
  explicit Init_UBXRxmRawx_leap_s(::ublox_ubx_msgs::msg::UBXRxmRawx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRawx_num_meas leap_s(::ublox_ubx_msgs::msg::UBXRxmRawx::_leap_s_type arg)
  {
    msg_.leap_s = std::move(arg);
    return Init_UBXRxmRawx_num_meas(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRawx msg_;
};

class Init_UBXRxmRawx_week
{
public:
  explicit Init_UBXRxmRawx_week(::ublox_ubx_msgs::msg::UBXRxmRawx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRawx_leap_s week(::ublox_ubx_msgs::msg::UBXRxmRawx::_week_type arg)
  {
    msg_.week = std::move(arg);
    return Init_UBXRxmRawx_leap_s(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRawx msg_;
};

class Init_UBXRxmRawx_rcv_tow
{
public:
  explicit Init_UBXRxmRawx_rcv_tow(::ublox_ubx_msgs::msg::UBXRxmRawx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRawx_week rcv_tow(::ublox_ubx_msgs::msg::UBXRxmRawx::_rcv_tow_type arg)
  {
    msg_.rcv_tow = std::move(arg);
    return Init_UBXRxmRawx_week(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRawx msg_;
};

class Init_UBXRxmRawx_header
{
public:
  Init_UBXRxmRawx_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXRxmRawx_rcv_tow header(::ublox_ubx_msgs::msg::UBXRxmRawx::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXRxmRawx_rcv_tow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRawx msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXRxmRawx>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXRxmRawx_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__BUILDER_HPP_
