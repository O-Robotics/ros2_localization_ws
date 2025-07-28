// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPosLLH.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_LLH__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_LLH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_pos_llh__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavPosLLH_v_acc
{
public:
  explicit Init_UBXNavPosLLH_v_acc(::ublox_ubx_msgs::msg::UBXNavPosLLH & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavPosLLH v_acc(::ublox_ubx_msgs::msg::UBXNavPosLLH::_v_acc_type arg)
  {
    msg_.v_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosLLH msg_;
};

class Init_UBXNavPosLLH_h_acc
{
public:
  explicit Init_UBXNavPosLLH_h_acc(::ublox_ubx_msgs::msg::UBXNavPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosLLH_v_acc h_acc(::ublox_ubx_msgs::msg::UBXNavPosLLH::_h_acc_type arg)
  {
    msg_.h_acc = std::move(arg);
    return Init_UBXNavPosLLH_v_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosLLH msg_;
};

class Init_UBXNavPosLLH_hmsl
{
public:
  explicit Init_UBXNavPosLLH_hmsl(::ublox_ubx_msgs::msg::UBXNavPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosLLH_h_acc hmsl(::ublox_ubx_msgs::msg::UBXNavPosLLH::_hmsl_type arg)
  {
    msg_.hmsl = std::move(arg);
    return Init_UBXNavPosLLH_h_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosLLH msg_;
};

class Init_UBXNavPosLLH_height
{
public:
  explicit Init_UBXNavPosLLH_height(::ublox_ubx_msgs::msg::UBXNavPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosLLH_hmsl height(::ublox_ubx_msgs::msg::UBXNavPosLLH::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_UBXNavPosLLH_hmsl(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosLLH msg_;
};

class Init_UBXNavPosLLH_lat
{
public:
  explicit Init_UBXNavPosLLH_lat(::ublox_ubx_msgs::msg::UBXNavPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosLLH_height lat(::ublox_ubx_msgs::msg::UBXNavPosLLH::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_UBXNavPosLLH_height(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosLLH msg_;
};

class Init_UBXNavPosLLH_lon
{
public:
  explicit Init_UBXNavPosLLH_lon(::ublox_ubx_msgs::msg::UBXNavPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosLLH_lat lon(::ublox_ubx_msgs::msg::UBXNavPosLLH::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_UBXNavPosLLH_lat(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosLLH msg_;
};

class Init_UBXNavPosLLH_itow
{
public:
  explicit Init_UBXNavPosLLH_itow(::ublox_ubx_msgs::msg::UBXNavPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavPosLLH_lon itow(::ublox_ubx_msgs::msg::UBXNavPosLLH::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavPosLLH_lon(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosLLH msg_;
};

class Init_UBXNavPosLLH_header
{
public:
  Init_UBXNavPosLLH_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavPosLLH_itow header(::ublox_ubx_msgs::msg::UBXNavPosLLH::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavPosLLH_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPosLLH msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavPosLLH>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavPosLLH_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_LLH__BUILDER_HPP_
