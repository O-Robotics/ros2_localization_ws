// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/OrbAlmInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_ALM_INFO__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_ALM_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/orb_alm_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_OrbAlmInfo_alm_source
{
public:
  explicit Init_OrbAlmInfo_alm_source(::ublox_ubx_msgs::msg::OrbAlmInfo & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::OrbAlmInfo alm_source(::ublox_ubx_msgs::msg::OrbAlmInfo::_alm_source_type arg)
  {
    msg_.alm_source = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbAlmInfo msg_;
};

class Init_OrbAlmInfo_alm_usability
{
public:
  Init_OrbAlmInfo_alm_usability()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrbAlmInfo_alm_source alm_usability(::ublox_ubx_msgs::msg::OrbAlmInfo::_alm_usability_type arg)
  {
    msg_.alm_usability = std::move(arg);
    return Init_OrbAlmInfo_alm_source(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbAlmInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::OrbAlmInfo>()
{
  return ublox_ubx_msgs::msg::builder::Init_OrbAlmInfo_alm_usability();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_ALM_INFO__BUILDER_HPP_
