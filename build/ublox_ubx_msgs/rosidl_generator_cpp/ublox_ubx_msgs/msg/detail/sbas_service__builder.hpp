// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/SBASService.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/sbas_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_SBASService_bad
{
public:
  explicit Init_SBASService_bad(::ublox_ubx_msgs::msg::SBASService & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::SBASService bad(::ublox_ubx_msgs::msg::SBASService::_bad_type arg)
  {
    msg_.bad = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASService msg_;
};

class Init_SBASService_test_mode
{
public:
  explicit Init_SBASService_test_mode(::ublox_ubx_msgs::msg::SBASService & msg)
  : msg_(msg)
  {}
  Init_SBASService_bad test_mode(::ublox_ubx_msgs::msg::SBASService::_test_mode_type arg)
  {
    msg_.test_mode = std::move(arg);
    return Init_SBASService_bad(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASService msg_;
};

class Init_SBASService_integrity
{
public:
  explicit Init_SBASService_integrity(::ublox_ubx_msgs::msg::SBASService & msg)
  : msg_(msg)
  {}
  Init_SBASService_test_mode integrity(::ublox_ubx_msgs::msg::SBASService::_integrity_type arg)
  {
    msg_.integrity = std::move(arg);
    return Init_SBASService_test_mode(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASService msg_;
};

class Init_SBASService_corrections
{
public:
  explicit Init_SBASService_corrections(::ublox_ubx_msgs::msg::SBASService & msg)
  : msg_(msg)
  {}
  Init_SBASService_integrity corrections(::ublox_ubx_msgs::msg::SBASService::_corrections_type arg)
  {
    msg_.corrections = std::move(arg);
    return Init_SBASService_integrity(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASService msg_;
};

class Init_SBASService_ranging
{
public:
  Init_SBASService_ranging()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SBASService_corrections ranging(::ublox_ubx_msgs::msg::SBASService::_ranging_type arg)
  {
    msg_.ranging = std::move(arg);
    return Init_SBASService_corrections(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASService msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::SBASService>()
{
  return ublox_ubx_msgs::msg::builder::Init_SBASService_ranging();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__BUILDER_HPP_
