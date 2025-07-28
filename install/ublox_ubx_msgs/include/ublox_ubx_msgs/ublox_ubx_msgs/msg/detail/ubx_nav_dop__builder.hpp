// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavDOP.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_dop__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavDOP_e_dop
{
public:
  explicit Init_UBXNavDOP_e_dop(::ublox_ubx_msgs::msg::UBXNavDOP & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavDOP e_dop(::ublox_ubx_msgs::msg::UBXNavDOP::_e_dop_type arg)
  {
    msg_.e_dop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavDOP msg_;
};

class Init_UBXNavDOP_n_dop
{
public:
  explicit Init_UBXNavDOP_n_dop(::ublox_ubx_msgs::msg::UBXNavDOP & msg)
  : msg_(msg)
  {}
  Init_UBXNavDOP_e_dop n_dop(::ublox_ubx_msgs::msg::UBXNavDOP::_n_dop_type arg)
  {
    msg_.n_dop = std::move(arg);
    return Init_UBXNavDOP_e_dop(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavDOP msg_;
};

class Init_UBXNavDOP_h_dop
{
public:
  explicit Init_UBXNavDOP_h_dop(::ublox_ubx_msgs::msg::UBXNavDOP & msg)
  : msg_(msg)
  {}
  Init_UBXNavDOP_n_dop h_dop(::ublox_ubx_msgs::msg::UBXNavDOP::_h_dop_type arg)
  {
    msg_.h_dop = std::move(arg);
    return Init_UBXNavDOP_n_dop(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavDOP msg_;
};

class Init_UBXNavDOP_v_dop
{
public:
  explicit Init_UBXNavDOP_v_dop(::ublox_ubx_msgs::msg::UBXNavDOP & msg)
  : msg_(msg)
  {}
  Init_UBXNavDOP_h_dop v_dop(::ublox_ubx_msgs::msg::UBXNavDOP::_v_dop_type arg)
  {
    msg_.v_dop = std::move(arg);
    return Init_UBXNavDOP_h_dop(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavDOP msg_;
};

class Init_UBXNavDOP_t_dop
{
public:
  explicit Init_UBXNavDOP_t_dop(::ublox_ubx_msgs::msg::UBXNavDOP & msg)
  : msg_(msg)
  {}
  Init_UBXNavDOP_v_dop t_dop(::ublox_ubx_msgs::msg::UBXNavDOP::_t_dop_type arg)
  {
    msg_.t_dop = std::move(arg);
    return Init_UBXNavDOP_v_dop(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavDOP msg_;
};

class Init_UBXNavDOP_p_dop
{
public:
  explicit Init_UBXNavDOP_p_dop(::ublox_ubx_msgs::msg::UBXNavDOP & msg)
  : msg_(msg)
  {}
  Init_UBXNavDOP_t_dop p_dop(::ublox_ubx_msgs::msg::UBXNavDOP::_p_dop_type arg)
  {
    msg_.p_dop = std::move(arg);
    return Init_UBXNavDOP_t_dop(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavDOP msg_;
};

class Init_UBXNavDOP_g_dop
{
public:
  explicit Init_UBXNavDOP_g_dop(::ublox_ubx_msgs::msg::UBXNavDOP & msg)
  : msg_(msg)
  {}
  Init_UBXNavDOP_p_dop g_dop(::ublox_ubx_msgs::msg::UBXNavDOP::_g_dop_type arg)
  {
    msg_.g_dop = std::move(arg);
    return Init_UBXNavDOP_p_dop(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavDOP msg_;
};

class Init_UBXNavDOP_itow
{
public:
  explicit Init_UBXNavDOP_itow(::ublox_ubx_msgs::msg::UBXNavDOP & msg)
  : msg_(msg)
  {}
  Init_UBXNavDOP_g_dop itow(::ublox_ubx_msgs::msg::UBXNavDOP::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavDOP_g_dop(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavDOP msg_;
};

class Init_UBXNavDOP_header
{
public:
  Init_UBXNavDOP_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavDOP_itow header(::ublox_ubx_msgs::msg::UBXNavDOP::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavDOP_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavDOP msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavDOP>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavDOP_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__BUILDER_HPP_
