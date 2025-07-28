// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/SatInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/sat_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_SatInfo_flags
{
public:
  explicit Init_SatInfo_flags(::ublox_ubx_msgs::msg::SatInfo & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::SatInfo flags(::ublox_ubx_msgs::msg::SatInfo::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatInfo msg_;
};

class Init_SatInfo_pr_res
{
public:
  explicit Init_SatInfo_pr_res(::ublox_ubx_msgs::msg::SatInfo & msg)
  : msg_(msg)
  {}
  Init_SatInfo_flags pr_res(::ublox_ubx_msgs::msg::SatInfo::_pr_res_type arg)
  {
    msg_.pr_res = std::move(arg);
    return Init_SatInfo_flags(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatInfo msg_;
};

class Init_SatInfo_azim
{
public:
  explicit Init_SatInfo_azim(::ublox_ubx_msgs::msg::SatInfo & msg)
  : msg_(msg)
  {}
  Init_SatInfo_pr_res azim(::ublox_ubx_msgs::msg::SatInfo::_azim_type arg)
  {
    msg_.azim = std::move(arg);
    return Init_SatInfo_pr_res(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatInfo msg_;
};

class Init_SatInfo_elev
{
public:
  explicit Init_SatInfo_elev(::ublox_ubx_msgs::msg::SatInfo & msg)
  : msg_(msg)
  {}
  Init_SatInfo_azim elev(::ublox_ubx_msgs::msg::SatInfo::_elev_type arg)
  {
    msg_.elev = std::move(arg);
    return Init_SatInfo_azim(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatInfo msg_;
};

class Init_SatInfo_cno
{
public:
  explicit Init_SatInfo_cno(::ublox_ubx_msgs::msg::SatInfo & msg)
  : msg_(msg)
  {}
  Init_SatInfo_elev cno(::ublox_ubx_msgs::msg::SatInfo::_cno_type arg)
  {
    msg_.cno = std::move(arg);
    return Init_SatInfo_elev(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatInfo msg_;
};

class Init_SatInfo_sv_id
{
public:
  explicit Init_SatInfo_sv_id(::ublox_ubx_msgs::msg::SatInfo & msg)
  : msg_(msg)
  {}
  Init_SatInfo_cno sv_id(::ublox_ubx_msgs::msg::SatInfo::_sv_id_type arg)
  {
    msg_.sv_id = std::move(arg);
    return Init_SatInfo_cno(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatInfo msg_;
};

class Init_SatInfo_gnss_id
{
public:
  Init_SatInfo_gnss_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SatInfo_sv_id gnss_id(::ublox_ubx_msgs::msg::SatInfo::_gnss_id_type arg)
  {
    msg_.gnss_id = std::move(arg);
    return Init_SatInfo_sv_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::SatInfo>()
{
  return ublox_ubx_msgs::msg::builder::Init_SatInfo_gnss_id();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__BUILDER_HPP_
