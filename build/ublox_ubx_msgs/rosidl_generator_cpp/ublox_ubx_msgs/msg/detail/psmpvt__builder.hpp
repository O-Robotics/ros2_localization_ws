// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/PSMPVT.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__PSMPVT__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__PSMPVT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/psmpvt__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_PSMPVT_state
{
public:
  Init_PSMPVT_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ublox_ubx_msgs::msg::PSMPVT state(::ublox_ubx_msgs::msg::PSMPVT::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::PSMPVT msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::PSMPVT>()
{
  return ublox_ubx_msgs::msg::builder::Init_PSMPVT_state();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__PSMPVT__BUILDER_HPP_
