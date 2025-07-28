// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavVelNED.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_vel_ned__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavVelNED_c_acc
{
public:
  explicit Init_UBXNavVelNED_c_acc(::ublox_ubx_msgs::msg::UBXNavVelNED & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavVelNED c_acc(::ublox_ubx_msgs::msg::UBXNavVelNED::_c_acc_type arg)
  {
    msg_.c_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

class Init_UBXNavVelNED_s_acc
{
public:
  explicit Init_UBXNavVelNED_s_acc(::ublox_ubx_msgs::msg::UBXNavVelNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelNED_c_acc s_acc(::ublox_ubx_msgs::msg::UBXNavVelNED::_s_acc_type arg)
  {
    msg_.s_acc = std::move(arg);
    return Init_UBXNavVelNED_c_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

class Init_UBXNavVelNED_heading
{
public:
  explicit Init_UBXNavVelNED_heading(::ublox_ubx_msgs::msg::UBXNavVelNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelNED_s_acc heading(::ublox_ubx_msgs::msg::UBXNavVelNED::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_UBXNavVelNED_s_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

class Init_UBXNavVelNED_g_speed
{
public:
  explicit Init_UBXNavVelNED_g_speed(::ublox_ubx_msgs::msg::UBXNavVelNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelNED_heading g_speed(::ublox_ubx_msgs::msg::UBXNavVelNED::_g_speed_type arg)
  {
    msg_.g_speed = std::move(arg);
    return Init_UBXNavVelNED_heading(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

class Init_UBXNavVelNED_speed
{
public:
  explicit Init_UBXNavVelNED_speed(::ublox_ubx_msgs::msg::UBXNavVelNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelNED_g_speed speed(::ublox_ubx_msgs::msg::UBXNavVelNED::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_UBXNavVelNED_g_speed(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

class Init_UBXNavVelNED_vel_d
{
public:
  explicit Init_UBXNavVelNED_vel_d(::ublox_ubx_msgs::msg::UBXNavVelNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelNED_speed vel_d(::ublox_ubx_msgs::msg::UBXNavVelNED::_vel_d_type arg)
  {
    msg_.vel_d = std::move(arg);
    return Init_UBXNavVelNED_speed(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

class Init_UBXNavVelNED_vel_e
{
public:
  explicit Init_UBXNavVelNED_vel_e(::ublox_ubx_msgs::msg::UBXNavVelNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelNED_vel_d vel_e(::ublox_ubx_msgs::msg::UBXNavVelNED::_vel_e_type arg)
  {
    msg_.vel_e = std::move(arg);
    return Init_UBXNavVelNED_vel_d(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

class Init_UBXNavVelNED_vel_n
{
public:
  explicit Init_UBXNavVelNED_vel_n(::ublox_ubx_msgs::msg::UBXNavVelNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelNED_vel_e vel_n(::ublox_ubx_msgs::msg::UBXNavVelNED::_vel_n_type arg)
  {
    msg_.vel_n = std::move(arg);
    return Init_UBXNavVelNED_vel_e(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

class Init_UBXNavVelNED_itow
{
public:
  explicit Init_UBXNavVelNED_itow(::ublox_ubx_msgs::msg::UBXNavVelNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelNED_vel_n itow(::ublox_ubx_msgs::msg::UBXNavVelNED::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavVelNED_vel_n(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

class Init_UBXNavVelNED_header
{
public:
  Init_UBXNavVelNED_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavVelNED_itow header(::ublox_ubx_msgs::msg::UBXNavVelNED::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavVelNED_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelNED msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavVelNED>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavVelNED_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__BUILDER_HPP_
