// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/OrbSVFlag.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/orb_sv_flag__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_OrbSVFlag_visibility
{
public:
  explicit Init_OrbSVFlag_visibility(::ublox_ubx_msgs::msg::OrbSVFlag & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::OrbSVFlag visibility(::ublox_ubx_msgs::msg::OrbSVFlag::_visibility_type arg)
  {
    msg_.visibility = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbSVFlag msg_;
};

class Init_OrbSVFlag_health
{
public:
  Init_OrbSVFlag_health()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrbSVFlag_visibility health(::ublox_ubx_msgs::msg::OrbSVFlag::_health_type arg)
  {
    msg_.health = std::move(arg);
    return Init_OrbSVFlag_visibility(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbSVFlag msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::OrbSVFlag>()
{
  return ublox_ubx_msgs::msg::builder::Init_OrbSVFlag_health();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__BUILDER_HPP_
