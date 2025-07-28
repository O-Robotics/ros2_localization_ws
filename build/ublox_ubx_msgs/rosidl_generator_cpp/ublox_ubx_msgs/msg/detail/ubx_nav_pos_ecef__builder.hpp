// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPosECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_pos_ecef__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavPosECEF_p_acc
{
public:
  explicit Init_UBXNavPosECEF_p_acc(::ublox_ubx_msgs::msg::UBXNavPosECEF & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavPosECEF p_acc(::ublox_ubx_msgs::msg::UBXNavPosECEF::_p_acc_type arg)
  {
    msg_.p_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosECEF msg_;
};

class Init_UBXNavPosECEF_ecef_z
{
public:
  explicit Init_UBXNavPosECEF_ecef_z(::ublox_ubx_msgs::msg::UBXNavPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosECEF_p_acc ecef_z(::ublox_ubx_msgs::msg::UBXNavPosECEF::_ecef_z_type arg)
  {
    msg_.ecef_z = std::move(arg);
    return Init_UBXNavPosECEF_p_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosECEF msg_;
};

class Init_UBXNavPosECEF_ecef_y
{
public:
  explicit Init_UBXNavPosECEF_ecef_y(::ublox_ubx_msgs::msg::UBXNavPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosECEF_ecef_z ecef_y(::ublox_ubx_msgs::msg::UBXNavPosECEF::_ecef_y_type arg)
  {
    msg_.ecef_y = std::move(arg);
    return Init_UBXNavPosECEF_ecef_z(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosECEF msg_;
};

class Init_UBXNavPosECEF_ecef_x
{
public:
  explicit Init_UBXNavPosECEF_ecef_x(::ublox_ubx_msgs::msg::UBXNavPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosECEF_ecef_y ecef_x(::ublox_ubx_msgs::msg::UBXNavPosECEF::_ecef_x_type arg)
  {
    msg_.ecef_x = std::move(arg);
    return Init_UBXNavPosECEF_ecef_y(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosECEF msg_;
};

class Init_UBXNavPosECEF_itow
{
public:
  explicit Init_UBXNavPosECEF_itow(::ublox_ubx_msgs::msg::UBXNavPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosECEF_ecef_x itow(::ublox_ubx_msgs::msg::UBXNavPosECEF::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavPosECEF_ecef_x(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosECEF msg_;
};

class Init_UBXNavPosECEF_header
{
public:
  Init_UBXNavPosECEF_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavPosECEF_itow header(::ublox_ubx_msgs::msg::UBXNavPosECEF::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavPosECEF_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosECEF msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavPosECEF>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavPosECEF_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__BUILDER_HPP_
