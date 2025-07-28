// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosLLH.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_llh__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavHPPosLLH_v_acc
{
public:
  explicit Init_UBXNavHPPosLLH_v_acc(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH v_acc(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_v_acc_type arg)
  {
    msg_.v_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_h_acc
{
public:
  explicit Init_UBXNavHPPosLLH_h_acc(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_v_acc h_acc(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_h_acc_type arg)
  {
    msg_.h_acc = std::move(arg);
    return Init_UBXNavHPPosLLH_v_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_hmsl_hp
{
public:
  explicit Init_UBXNavHPPosLLH_hmsl_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_h_acc hmsl_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_hmsl_hp_type arg)
  {
    msg_.hmsl_hp = std::move(arg);
    return Init_UBXNavHPPosLLH_h_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_height_hp
{
public:
  explicit Init_UBXNavHPPosLLH_height_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_hmsl_hp height_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_height_hp_type arg)
  {
    msg_.height_hp = std::move(arg);
    return Init_UBXNavHPPosLLH_hmsl_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_lat_hp
{
public:
  explicit Init_UBXNavHPPosLLH_lat_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_height_hp lat_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_lat_hp_type arg)
  {
    msg_.lat_hp = std::move(arg);
    return Init_UBXNavHPPosLLH_height_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_lon_hp
{
public:
  explicit Init_UBXNavHPPosLLH_lon_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_lat_hp lon_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_lon_hp_type arg)
  {
    msg_.lon_hp = std::move(arg);
    return Init_UBXNavHPPosLLH_lat_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_hmsl
{
public:
  explicit Init_UBXNavHPPosLLH_hmsl(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_lon_hp hmsl(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_hmsl_type arg)
  {
    msg_.hmsl = std::move(arg);
    return Init_UBXNavHPPosLLH_lon_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_height
{
public:
  explicit Init_UBXNavHPPosLLH_height(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_hmsl height(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_UBXNavHPPosLLH_hmsl(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_lat
{
public:
  explicit Init_UBXNavHPPosLLH_lat(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_height lat(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_UBXNavHPPosLLH_height(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_lon
{
public:
  explicit Init_UBXNavHPPosLLH_lon(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_lat lon(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_UBXNavHPPosLLH_lat(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_itow
{
public:
  explicit Init_UBXNavHPPosLLH_itow(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_lon itow(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavHPPosLLH_lon(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_invalid_hmsl_hp
{
public:
  explicit Init_UBXNavHPPosLLH_invalid_hmsl_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_itow invalid_hmsl_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_invalid_hmsl_hp_type arg)
  {
    msg_.invalid_hmsl_hp = std::move(arg);
    return Init_UBXNavHPPosLLH_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_invalid_height_hp
{
public:
  explicit Init_UBXNavHPPosLLH_invalid_height_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_invalid_hmsl_hp invalid_height_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_invalid_height_hp_type arg)
  {
    msg_.invalid_height_hp = std::move(arg);
    return Init_UBXNavHPPosLLH_invalid_hmsl_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_invalid_lat_hp
{
public:
  explicit Init_UBXNavHPPosLLH_invalid_lat_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_invalid_height_hp invalid_lat_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_invalid_lat_hp_type arg)
  {
    msg_.invalid_lat_hp = std::move(arg);
    return Init_UBXNavHPPosLLH_invalid_height_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_invalid_lon_hp
{
public:
  explicit Init_UBXNavHPPosLLH_invalid_lon_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_invalid_lat_hp invalid_lon_hp(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_invalid_lon_hp_type arg)
  {
    msg_.invalid_lon_hp = std::move(arg);
    return Init_UBXNavHPPosLLH_invalid_lat_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_invalid_hmsl
{
public:
  explicit Init_UBXNavHPPosLLH_invalid_hmsl(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_invalid_lon_hp invalid_hmsl(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_invalid_hmsl_type arg)
  {
    msg_.invalid_hmsl = std::move(arg);
    return Init_UBXNavHPPosLLH_invalid_lon_hp(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_invalid_height
{
public:
  explicit Init_UBXNavHPPosLLH_invalid_height(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_invalid_hmsl invalid_height(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_invalid_height_type arg)
  {
    msg_.invalid_height = std::move(arg);
    return Init_UBXNavHPPosLLH_invalid_hmsl(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_invalid_lat
{
public:
  explicit Init_UBXNavHPPosLLH_invalid_lat(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_invalid_height invalid_lat(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_invalid_lat_type arg)
  {
    msg_.invalid_lat = std::move(arg);
    return Init_UBXNavHPPosLLH_invalid_height(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_invalid_lon
{
public:
  explicit Init_UBXNavHPPosLLH_invalid_lon(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_invalid_lat invalid_lon(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_invalid_lon_type arg)
  {
    msg_.invalid_lon = std::move(arg);
    return Init_UBXNavHPPosLLH_invalid_lat(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_version
{
public:
  explicit Init_UBXNavHPPosLLH_version(::ublox_ubx_msgs::msg::UBXNavHPPosLLH & msg)
  : msg_(msg)
  {}
  Init_UBXNavHPPosLLH_invalid_lon version(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXNavHPPosLLH_invalid_lon(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

class Init_UBXNavHPPosLLH_header
{
public:
  Init_UBXNavHPPosLLH_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavHPPosLLH_version header(::ublox_ubx_msgs::msg::UBXNavHPPosLLH::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavHPPosLLH_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavHPPosLLH msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavHPPosLLH>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavHPPosLLH_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__BUILDER_HPP_
