// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXSecSigLog.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG_LOG__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG_LOG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_sec_sig_log__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXSecSigLog_events
{
public:
  explicit Init_UBXSecSigLog_events(::ublox_ubx_msgs::msg::UBXSecSigLog & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXSecSigLog events(::ublox_ubx_msgs::msg::UBXSecSigLog::_events_type arg)
  {
    msg_.events = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSigLog msg_;
};

class Init_UBXSecSigLog_num_events
{
public:
  explicit Init_UBXSecSigLog_num_events(::ublox_ubx_msgs::msg::UBXSecSigLog & msg)
  : msg_(msg)
  {}
  Init_UBXSecSigLog_events num_events(::ublox_ubx_msgs::msg::UBXSecSigLog::_num_events_type arg)
  {
    msg_.num_events = std::move(arg);
    return Init_UBXSecSigLog_events(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSigLog msg_;
};

class Init_UBXSecSigLog_version
{
public:
  explicit Init_UBXSecSigLog_version(::ublox_ubx_msgs::msg::UBXSecSigLog & msg)
  : msg_(msg)
  {}
  Init_UBXSecSigLog_num_events version(::ublox_ubx_msgs::msg::UBXSecSigLog::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXSecSigLog_num_events(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSigLog msg_;
};

class Init_UBXSecSigLog_header
{
public:
  Init_UBXSecSigLog_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXSecSigLog_version header(::ublox_ubx_msgs::msg::UBXSecSigLog::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXSecSigLog_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSigLog msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXSecSigLog>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXSecSigLog_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG_LOG__BUILDER_HPP_
