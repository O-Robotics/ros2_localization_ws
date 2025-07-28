// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavOdo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ODO__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ODO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_odo__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavOdo_distance_std
{
public:
  explicit Init_UBXNavOdo_distance_std(::ublox_ubx_msgs::msg::UBXNavOdo & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavOdo distance_std(::ublox_ubx_msgs::msg::UBXNavOdo::_distance_std_type arg)
  {
    msg_.distance_std = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOdo msg_;
};

class Init_UBXNavOdo_total_distance
{
public:
  explicit Init_UBXNavOdo_total_distance(::ublox_ubx_msgs::msg::UBXNavOdo & msg)
  : msg_(msg)
  {}
  Init_UBXNavOdo_distance_std total_distance(::ublox_ubx_msgs::msg::UBXNavOdo::_total_distance_type arg)
  {
    msg_.total_distance = std::move(arg);
    return Init_UBXNavOdo_distance_std(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOdo msg_;
};

class Init_UBXNavOdo_distance
{
public:
  explicit Init_UBXNavOdo_distance(::ublox_ubx_msgs::msg::UBXNavOdo & msg)
  : msg_(msg)
  {}
  Init_UBXNavOdo_total_distance distance(::ublox_ubx_msgs::msg::UBXNavOdo::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_UBXNavOdo_total_distance(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOdo msg_;
};

class Init_UBXNavOdo_itow
{
public:
  explicit Init_UBXNavOdo_itow(::ublox_ubx_msgs::msg::UBXNavOdo & msg)
  : msg_(msg)
  {}
  Init_UBXNavOdo_distance itow(::ublox_ubx_msgs::msg::UBXNavOdo::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavOdo_distance(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOdo msg_;
};

class Init_UBXNavOdo_version
{
public:
  explicit Init_UBXNavOdo_version(::ublox_ubx_msgs::msg::UBXNavOdo & msg)
  : msg_(msg)
  {}
  Init_UBXNavOdo_itow version(::ublox_ubx_msgs::msg::UBXNavOdo::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXNavOdo_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOdo msg_;
};

class Init_UBXNavOdo_header
{
public:
  Init_UBXNavOdo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavOdo_version header(::ublox_ubx_msgs::msg::UBXNavOdo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavOdo_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavOdo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavOdo>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavOdo_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ODO__BUILDER_HPP_
