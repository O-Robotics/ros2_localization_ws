// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/RecStat.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__REC_STAT__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__REC_STAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/rec_stat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_RecStat_clk_reset
{
public:
  explicit Init_RecStat_clk_reset(::ublox_ubx_msgs::msg::RecStat & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::RecStat clk_reset(::ublox_ubx_msgs::msg::RecStat::_clk_reset_type arg)
  {
    msg_.clk_reset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RecStat msg_;
};

class Init_RecStat_leap_sec
{
public:
  Init_RecStat_leap_sec()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RecStat_clk_reset leap_sec(::ublox_ubx_msgs::msg::RecStat::_leap_sec_type arg)
  {
    msg_.leap_sec = std::move(arg);
    return Init_RecStat_clk_reset(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RecStat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::RecStat>()
{
  return ublox_ubx_msgs::msg::builder::Init_RecStat_leap_sec();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__REC_STAT__BUILDER_HPP_
