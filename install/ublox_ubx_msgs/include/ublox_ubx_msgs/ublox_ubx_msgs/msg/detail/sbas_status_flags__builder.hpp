// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/SBASStatusFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/sbas_status_flags__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_SBASStatusFlags_integrity_used
{
public:
  Init_SBASStatusFlags_integrity_used()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ublox_ubx_msgs::msg::SBASStatusFlags integrity_used(::ublox_ubx_msgs::msg::SBASStatusFlags::_integrity_used_type arg)
  {
    msg_.integrity_used = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASStatusFlags msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::SBASStatusFlags>()
{
  return ublox_ubx_msgs::msg::builder::Init_SBASStatusFlags_integrity_used();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__BUILDER_HPP_
