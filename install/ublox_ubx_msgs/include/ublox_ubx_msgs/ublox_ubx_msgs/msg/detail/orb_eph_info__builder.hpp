// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/OrbEphInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/orb_eph_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_OrbEphInfo_eph_source
{
public:
  explicit Init_OrbEphInfo_eph_source(::ublox_ubx_msgs::msg::OrbEphInfo & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::OrbEphInfo eph_source(::ublox_ubx_msgs::msg::OrbEphInfo::_eph_source_type arg)
  {
    msg_.eph_source = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbEphInfo msg_;
};

class Init_OrbEphInfo_eph_usability
{
public:
  Init_OrbEphInfo_eph_usability()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrbEphInfo_eph_source eph_usability(::ublox_ubx_msgs::msg::OrbEphInfo::_eph_usability_type arg)
  {
    msg_.eph_usability = std::move(arg);
    return Init_OrbEphInfo_eph_source(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbEphInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::OrbEphInfo>()
{
  return ublox_ubx_msgs::msg::builder::Init_OrbEphInfo_eph_usability();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__BUILDER_HPP_
