// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/OrbSVInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/orb_sv_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_OrbSVInfo_other_orb
{
public:
  explicit Init_OrbSVInfo_other_orb(::ublox_ubx_msgs::msg::OrbSVInfo & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::OrbSVInfo other_orb(::ublox_ubx_msgs::msg::OrbSVInfo::_other_orb_type arg)
  {
    msg_.other_orb = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbSVInfo msg_;
};

class Init_OrbSVInfo_alm
{
public:
  explicit Init_OrbSVInfo_alm(::ublox_ubx_msgs::msg::OrbSVInfo & msg)
  : msg_(msg)
  {}
  Init_OrbSVInfo_other_orb alm(::ublox_ubx_msgs::msg::OrbSVInfo::_alm_type arg)
  {
    msg_.alm = std::move(arg);
    return Init_OrbSVInfo_other_orb(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbSVInfo msg_;
};

class Init_OrbSVInfo_eph
{
public:
  explicit Init_OrbSVInfo_eph(::ublox_ubx_msgs::msg::OrbSVInfo & msg)
  : msg_(msg)
  {}
  Init_OrbSVInfo_alm eph(::ublox_ubx_msgs::msg::OrbSVInfo::_eph_type arg)
  {
    msg_.eph = std::move(arg);
    return Init_OrbSVInfo_alm(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbSVInfo msg_;
};

class Init_OrbSVInfo_sv_flag
{
public:
  explicit Init_OrbSVInfo_sv_flag(::ublox_ubx_msgs::msg::OrbSVInfo & msg)
  : msg_(msg)
  {}
  Init_OrbSVInfo_eph sv_flag(::ublox_ubx_msgs::msg::OrbSVInfo::_sv_flag_type arg)
  {
    msg_.sv_flag = std::move(arg);
    return Init_OrbSVInfo_eph(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbSVInfo msg_;
};

class Init_OrbSVInfo_sv_id
{
public:
  explicit Init_OrbSVInfo_sv_id(::ublox_ubx_msgs::msg::OrbSVInfo & msg)
  : msg_(msg)
  {}
  Init_OrbSVInfo_sv_flag sv_id(::ublox_ubx_msgs::msg::OrbSVInfo::_sv_id_type arg)
  {
    msg_.sv_id = std::move(arg);
    return Init_OrbSVInfo_sv_flag(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbSVInfo msg_;
};

class Init_OrbSVInfo_gnss_id
{
public:
  Init_OrbSVInfo_gnss_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrbSVInfo_sv_id gnss_id(::ublox_ubx_msgs::msg::OrbSVInfo::_gnss_id_type arg)
  {
    msg_.gnss_id = std::move(arg);
    return Init_OrbSVInfo_sv_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OrbSVInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::OrbSVInfo>()
{
  return ublox_ubx_msgs::msg::builder::Init_OrbSVInfo_gnss_id();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__BUILDER_HPP_
