// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavCov.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_cov__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavCov_vel_cov_dd
{
public:
  explicit Init_UBXNavCov_vel_cov_dd(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavCov vel_cov_dd(::ublox_ubx_msgs::msg::UBXNavCov::_vel_cov_dd_type arg)
  {
    msg_.vel_cov_dd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_vel_cov_ed
{
public:
  explicit Init_UBXNavCov_vel_cov_ed(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_vel_cov_dd vel_cov_ed(::ublox_ubx_msgs::msg::UBXNavCov::_vel_cov_ed_type arg)
  {
    msg_.vel_cov_ed = std::move(arg);
    return Init_UBXNavCov_vel_cov_dd(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_vel_cov_ee
{
public:
  explicit Init_UBXNavCov_vel_cov_ee(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_vel_cov_ed vel_cov_ee(::ublox_ubx_msgs::msg::UBXNavCov::_vel_cov_ee_type arg)
  {
    msg_.vel_cov_ee = std::move(arg);
    return Init_UBXNavCov_vel_cov_ed(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_vel_cov_nd
{
public:
  explicit Init_UBXNavCov_vel_cov_nd(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_vel_cov_ee vel_cov_nd(::ublox_ubx_msgs::msg::UBXNavCov::_vel_cov_nd_type arg)
  {
    msg_.vel_cov_nd = std::move(arg);
    return Init_UBXNavCov_vel_cov_ee(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_vel_cov_ne
{
public:
  explicit Init_UBXNavCov_vel_cov_ne(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_vel_cov_nd vel_cov_ne(::ublox_ubx_msgs::msg::UBXNavCov::_vel_cov_ne_type arg)
  {
    msg_.vel_cov_ne = std::move(arg);
    return Init_UBXNavCov_vel_cov_nd(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_vel_cov_nn
{
public:
  explicit Init_UBXNavCov_vel_cov_nn(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_vel_cov_ne vel_cov_nn(::ublox_ubx_msgs::msg::UBXNavCov::_vel_cov_nn_type arg)
  {
    msg_.vel_cov_nn = std::move(arg);
    return Init_UBXNavCov_vel_cov_ne(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_pos_cov_dd
{
public:
  explicit Init_UBXNavCov_pos_cov_dd(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_vel_cov_nn pos_cov_dd(::ublox_ubx_msgs::msg::UBXNavCov::_pos_cov_dd_type arg)
  {
    msg_.pos_cov_dd = std::move(arg);
    return Init_UBXNavCov_vel_cov_nn(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_pos_cov_ed
{
public:
  explicit Init_UBXNavCov_pos_cov_ed(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_pos_cov_dd pos_cov_ed(::ublox_ubx_msgs::msg::UBXNavCov::_pos_cov_ed_type arg)
  {
    msg_.pos_cov_ed = std::move(arg);
    return Init_UBXNavCov_pos_cov_dd(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_pos_cov_ee
{
public:
  explicit Init_UBXNavCov_pos_cov_ee(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_pos_cov_ed pos_cov_ee(::ublox_ubx_msgs::msg::UBXNavCov::_pos_cov_ee_type arg)
  {
    msg_.pos_cov_ee = std::move(arg);
    return Init_UBXNavCov_pos_cov_ed(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_pos_cov_nd
{
public:
  explicit Init_UBXNavCov_pos_cov_nd(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_pos_cov_ee pos_cov_nd(::ublox_ubx_msgs::msg::UBXNavCov::_pos_cov_nd_type arg)
  {
    msg_.pos_cov_nd = std::move(arg);
    return Init_UBXNavCov_pos_cov_ee(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_pos_cov_ne
{
public:
  explicit Init_UBXNavCov_pos_cov_ne(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_pos_cov_nd pos_cov_ne(::ublox_ubx_msgs::msg::UBXNavCov::_pos_cov_ne_type arg)
  {
    msg_.pos_cov_ne = std::move(arg);
    return Init_UBXNavCov_pos_cov_nd(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_pos_cov_nn
{
public:
  explicit Init_UBXNavCov_pos_cov_nn(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_pos_cov_ne pos_cov_nn(::ublox_ubx_msgs::msg::UBXNavCov::_pos_cov_nn_type arg)
  {
    msg_.pos_cov_nn = std::move(arg);
    return Init_UBXNavCov_pos_cov_ne(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_vel_cor_valid
{
public:
  explicit Init_UBXNavCov_vel_cor_valid(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_pos_cov_nn vel_cor_valid(::ublox_ubx_msgs::msg::UBXNavCov::_vel_cor_valid_type arg)
  {
    msg_.vel_cor_valid = std::move(arg);
    return Init_UBXNavCov_pos_cov_nn(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_pos_cor_valid
{
public:
  explicit Init_UBXNavCov_pos_cor_valid(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_vel_cor_valid pos_cor_valid(::ublox_ubx_msgs::msg::UBXNavCov::_pos_cor_valid_type arg)
  {
    msg_.pos_cor_valid = std::move(arg);
    return Init_UBXNavCov_vel_cor_valid(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_version
{
public:
  explicit Init_UBXNavCov_version(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_pos_cor_valid version(::ublox_ubx_msgs::msg::UBXNavCov::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXNavCov_pos_cor_valid(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_itow
{
public:
  explicit Init_UBXNavCov_itow(::ublox_ubx_msgs::msg::UBXNavCov & msg)
  : msg_(msg)
  {}
  Init_UBXNavCov_version itow(::ublox_ubx_msgs::msg::UBXNavCov::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavCov_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

class Init_UBXNavCov_header
{
public:
  Init_UBXNavCov_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavCov_itow header(::ublox_ubx_msgs::msg::UBXNavCov::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavCov_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavCov msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavCov>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavCov_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__BUILDER_HPP_
