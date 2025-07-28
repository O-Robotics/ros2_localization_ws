// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavHPPosECEF_p_acc
{
public:
  explicit Init_UBXNavHPPosECEF_p_acc(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF p_acc(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_p_acc_type arg)
  {
    msg_.p_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_invalid_ecef_z_hp
{
public:
  explicit Init_UBXNavHPPosECEF_invalid_ecef_z_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_p_acc invalid_ecef_z_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_invalid_ecef_z_hp_type arg)
  {
    msg_.invalid_ecef_z_hp = std::move(arg);
    return Init_UBXNavHPPosECEF_p_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_invalid_ecef_y_hp
{
public:
  explicit Init_UBXNavHPPosECEF_invalid_ecef_y_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_invalid_ecef_z_hp invalid_ecef_y_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_invalid_ecef_y_hp_type arg)
  {
    msg_.invalid_ecef_y_hp = std::move(arg);
    return Init_UBXNavHPPosECEF_invalid_ecef_z_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_invalid_ecef_x_hp
{
public:
  explicit Init_UBXNavHPPosECEF_invalid_ecef_x_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_invalid_ecef_y_hp invalid_ecef_x_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_invalid_ecef_x_hp_type arg)
  {
    msg_.invalid_ecef_x_hp = std::move(arg);
    return Init_UBXNavHPPosECEF_invalid_ecef_y_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_invalid_ecef_z
{
public:
  explicit Init_UBXNavHPPosECEF_invalid_ecef_z(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_invalid_ecef_x_hp invalid_ecef_z(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_invalid_ecef_z_type arg)
  {
    msg_.invalid_ecef_z = std::move(arg);
    return Init_UBXNavHPPosECEF_invalid_ecef_x_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_invalid_ecef_y
{
public:
  explicit Init_UBXNavHPPosECEF_invalid_ecef_y(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_invalid_ecef_z invalid_ecef_y(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_invalid_ecef_y_type arg)
  {
    msg_.invalid_ecef_y = std::move(arg);
    return Init_UBXNavHPPosECEF_invalid_ecef_z(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_invalid_ecef_x
{
public:
  explicit Init_UBXNavHPPosECEF_invalid_ecef_x(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_invalid_ecef_y invalid_ecef_x(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_invalid_ecef_x_type arg)
  {
    msg_.invalid_ecef_x = std::move(arg);
    return Init_UBXNavHPPosECEF_invalid_ecef_y(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_ecef_z_hp
{
public:
  explicit Init_UBXNavHPPosECEF_ecef_z_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_invalid_ecef_x ecef_z_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_ecef_z_hp_type arg)
  {
    msg_.ecef_z_hp = std::move(arg);
    return Init_UBXNavHPPosECEF_invalid_ecef_x(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_ecef_y_hp
{
public:
  explicit Init_UBXNavHPPosECEF_ecef_y_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_ecef_z_hp ecef_y_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_ecef_y_hp_type arg)
  {
    msg_.ecef_y_hp = std::move(arg);
    return Init_UBXNavHPPosECEF_ecef_z_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_ecef_x_hp
{
public:
  explicit Init_UBXNavHPPosECEF_ecef_x_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_ecef_y_hp ecef_x_hp(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_ecef_x_hp_type arg)
  {
    msg_.ecef_x_hp = std::move(arg);
    return Init_UBXNavHPPosECEF_ecef_y_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_ecef_z
{
public:
  explicit Init_UBXNavHPPosECEF_ecef_z(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_ecef_x_hp ecef_z(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_ecef_z_type arg)
  {
    msg_.ecef_z = std::move(arg);
    return Init_UBXNavHPPosECEF_ecef_x_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_ecef_y
{
public:
  explicit Init_UBXNavHPPosECEF_ecef_y(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_ecef_z ecef_y(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_ecef_y_type arg)
  {
    msg_.ecef_y = std::move(arg);
    return Init_UBXNavHPPosECEF_ecef_z(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_ecef_x
{
public:
  explicit Init_UBXNavHPPosECEF_ecef_x(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_ecef_y ecef_x(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_ecef_x_type arg)
  {
    msg_.ecef_x = std::move(arg);
    return Init_UBXNavHPPosECEF_ecef_y(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_itow
{
public:
  explicit Init_UBXNavHPPosECEF_itow(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_ecef_x itow(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavHPPosECEF_ecef_x(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_version
{
public:
  explicit Init_UBXNavHPPosECEF_version(::ublox_ubx_msgs::msg::UBXNavHPPosECEF & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosECEF_itow version(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXNavHPPosECEF_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

class Init_UBXNavHPPosECEF_header
{
public:
  Init_UBXNavHPPosECEF_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavHPPosECEF_version header(::ublox_ubx_msgs::msg::UBXNavHPPosECEF::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavHPPosECEF_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosECEF msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavHPPosECEF>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavHPPosECEF_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__BUILDER_HPP_
