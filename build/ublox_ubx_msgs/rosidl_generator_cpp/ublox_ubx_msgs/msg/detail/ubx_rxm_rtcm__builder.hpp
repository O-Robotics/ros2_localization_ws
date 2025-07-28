// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmRTCM.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rtcm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXRxmRTCM_msg_type
{
public:
  explicit Init_UBXRxmRTCM_msg_type(::ublox_ubx_msgs::msg::UBXRxmRTCM & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXRxmRTCM msg_type(::ublox_ubx_msgs::msg::UBXRxmRTCM::_msg_type_type arg)
  {
    msg_.msg_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRTCM msg_;
};

class Init_UBXRxmRTCM_ref_station
{
public:
  explicit Init_UBXRxmRTCM_ref_station(::ublox_ubx_msgs::msg::UBXRxmRTCM & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRTCM_msg_type ref_station(::ublox_ubx_msgs::msg::UBXRxmRTCM::_ref_station_type arg)
  {
    msg_.ref_station = std::move(arg);
    return Init_UBXRxmRTCM_msg_type(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRTCM msg_;
};

class Init_UBXRxmRTCM_sub_type
{
public:
  explicit Init_UBXRxmRTCM_sub_type(::ublox_ubx_msgs::msg::UBXRxmRTCM & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRTCM_ref_station sub_type(::ublox_ubx_msgs::msg::UBXRxmRTCM::_sub_type_type arg)
  {
    msg_.sub_type = std::move(arg);
    return Init_UBXRxmRTCM_ref_station(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRTCM msg_;
};

class Init_UBXRxmRTCM_msg_used
{
public:
  explicit Init_UBXRxmRTCM_msg_used(::ublox_ubx_msgs::msg::UBXRxmRTCM & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRTCM_sub_type msg_used(::ublox_ubx_msgs::msg::UBXRxmRTCM::_msg_used_type arg)
  {
    msg_.msg_used = std::move(arg);
    return Init_UBXRxmRTCM_sub_type(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRTCM msg_;
};

class Init_UBXRxmRTCM_crc_failed
{
public:
  explicit Init_UBXRxmRTCM_crc_failed(::ublox_ubx_msgs::msg::UBXRxmRTCM & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRTCM_msg_used crc_failed(::ublox_ubx_msgs::msg::UBXRxmRTCM::_crc_failed_type arg)
  {
    msg_.crc_failed = std::move(arg);
    return Init_UBXRxmRTCM_msg_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRTCM msg_;
};

class Init_UBXRxmRTCM_version
{
public:
  explicit Init_UBXRxmRTCM_version(::ublox_ubx_msgs::msg::UBXRxmRTCM & msg)
  : msg_(msg)
  {}
  Init_UBXRxmRTCM_crc_failed version(::ublox_ubx_msgs::msg::UBXRxmRTCM::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXRxmRTCM_crc_failed(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRTCM msg_;
};

class Init_UBXRxmRTCM_header
{
public:
  Init_UBXRxmRTCM_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXRxmRTCM_version header(::ublox_ubx_msgs::msg::UBXRxmRTCM::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXRxmRTCM_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmRTCM msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXRxmRTCM>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXRxmRTCM_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__BUILDER_HPP_
