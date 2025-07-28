// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/GpsFix.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__GPS_FIX__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__GPS_FIX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/gps_fix__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_GpsFix_fix_type
{
public:
  Init_GpsFix_fix_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ublox_ubx_msgs::msg::GpsFix fix_type(::ublox_ubx_msgs::msg::GpsFix::_fix_type_type arg)
  {
    msg_.fix_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::GpsFix msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::GpsFix>()
{
  return ublox_ubx_msgs::msg::builder::Init_GpsFix_fix_type();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__GPS_FIX__BUILDER_HPP_
