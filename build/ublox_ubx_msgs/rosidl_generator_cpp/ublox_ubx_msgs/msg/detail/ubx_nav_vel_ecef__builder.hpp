// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavVelECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_vel_ecef__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavVelECEF_s_acc
{
public:
  explicit Init_UBXNavVelECEF_s_acc(::ublox_ubx_msgs::msg::UBXNavVelECEF & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavVelECEF s_acc(::ublox_ubx_msgs::msg::UBXNavVelECEF::_s_acc_type arg)
  {
    msg_.s_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelECEF msg_;
};

class Init_UBXNavVelECEF_ecef_vz
{
public:
  explicit Init_UBXNavVelECEF_ecef_vz(::ublox_ubx_msgs::msg::UBXNavVelECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelECEF_s_acc ecef_vz(::ublox_ubx_msgs::msg::UBXNavVelECEF::_ecef_vz_type arg)
  {
    msg_.ecef_vz = std::move(arg);
    return Init_UBXNavVelECEF_s_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelECEF msg_;
};

class Init_UBXNavVelECEF_ecef_vy
{
public:
  explicit Init_UBXNavVelECEF_ecef_vy(::ublox_ubx_msgs::msg::UBXNavVelECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelECEF_ecef_vz ecef_vy(::ublox_ubx_msgs::msg::UBXNavVelECEF::_ecef_vy_type arg)
  {
    msg_.ecef_vy = std::move(arg);
    return Init_UBXNavVelECEF_ecef_vz(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelECEF msg_;
};

class Init_UBXNavVelECEF_ecef_vx
{
public:
  explicit Init_UBXNavVelECEF_ecef_vx(::ublox_ubx_msgs::msg::UBXNavVelECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelECEF_ecef_vy ecef_vx(::ublox_ubx_msgs::msg::UBXNavVelECEF::_ecef_vx_type arg)
  {
    msg_.ecef_vx = std::move(arg);
    return Init_UBXNavVelECEF_ecef_vy(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelECEF msg_;
};

class Init_UBXNavVelECEF_itow
{
public:
  explicit Init_UBXNavVelECEF_itow(::ublox_ubx_msgs::msg::UBXNavVelECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavVelECEF_ecef_vx itow(::ublox_ubx_msgs::msg::UBXNavVelECEF::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavVelECEF_ecef_vx(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelECEF msg_;
};

class Init_UBXNavVelECEF_header
{
public:
  Init_UBXNavVelECEF_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavVelECEF_itow header(::ublox_ubx_msgs::msg::UBXNavVelECEF::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavVelECEF_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavVelECEF msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavVelECEF>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavVelECEF_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__BUILDER_HPP_
