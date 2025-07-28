// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavClock.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_clock__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavClock_f_acc
{
public:
  explicit Init_UBXNavClock_f_acc(::ublox_ubx_msgs::msg::UBXNavClock & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavClock f_acc(::ublox_ubx_msgs::msg::UBXNavClock::_f_acc_type arg)
  {
    msg_.f_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavClock msg_;
};

class Init_UBXNavClock_t_acc
{
public:
  explicit Init_UBXNavClock_t_acc(::ublox_ubx_msgs::msg::UBXNavClock & msg)
  : msg_(msg)
  {}
  Init_UBXNavClock_f_acc t_acc(::ublox_ubx_msgs::msg::UBXNavClock::_t_acc_type arg)
  {
    msg_.t_acc = std::move(arg);
    return Init_UBXNavClock_f_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavClock msg_;
};

class Init_UBXNavClock_clk_d
{
public:
  explicit Init_UBXNavClock_clk_d(::ublox_ubx_msgs::msg::UBXNavClock & msg)
  : msg_(msg)
  {}
  Init_UBXNavClock_t_acc clk_d(::ublox_ubx_msgs::msg::UBXNavClock::_clk_d_type arg)
  {
    msg_.clk_d = std::move(arg);
    return Init_UBXNavClock_t_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavClock msg_;
};

class Init_UBXNavClock_clk_b
{
public:
  explicit Init_UBXNavClock_clk_b(::ublox_ubx_msgs::msg::UBXNavClock & msg)
  : msg_(msg)
  {}
  Init_UBXNavClock_clk_d clk_b(::ublox_ubx_msgs::msg::UBXNavClock::_clk_b_type arg)
  {
    msg_.clk_b = std::move(arg);
    return Init_UBXNavClock_clk_d(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavClock msg_;
};

class Init_UBXNavClock_itow
{
public:
  explicit Init_UBXNavClock_itow(::ublox_ubx_msgs::msg::UBXNavClock & msg)
  : msg_(msg)
  {}
  Init_UBXNavClock_clk_b itow(::ublox_ubx_msgs::msg::UBXNavClock::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavClock_clk_b(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavClock msg_;
};

class Init_UBXNavClock_header
{
public:
  Init_UBXNavClock_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavClock_itow header(::ublox_ubx_msgs::msg::UBXNavClock::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavClock_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavClock msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavClock>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavClock_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__BUILDER_HPP_
