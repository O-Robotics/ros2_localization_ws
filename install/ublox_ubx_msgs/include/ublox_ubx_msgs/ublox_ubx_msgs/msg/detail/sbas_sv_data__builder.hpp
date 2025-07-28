// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/SBASSvData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_SBASSvData_ic
{
public:
  explicit Init_SBASSvData_ic(::ublox_ubx_msgs::msg::SBASSvData & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::SBASSvData ic(::ublox_ubx_msgs::msg::SBASSvData::_ic_type arg)
  {
    msg_.ic = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASSvData msg_;
};

class Init_SBASSvData_reserved_3
{
public:
  explicit Init_SBASSvData_reserved_3(::ublox_ubx_msgs::msg::SBASSvData & msg)
  : msg_(msg)
  {}
  Init_SBASSvData_ic reserved_3(::ublox_ubx_msgs::msg::SBASSvData::_reserved_3_type arg)
  {
    msg_.reserved_3 = std::move(arg);
    return Init_SBASSvData_ic(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASSvData msg_;
};

class Init_SBASSvData_prc
{
public:
  explicit Init_SBASSvData_prc(::ublox_ubx_msgs::msg::SBASSvData & msg)
  : msg_(msg)
  {}
  Init_SBASSvData_reserved_3 prc(::ublox_ubx_msgs::msg::SBASSvData::_prc_type arg)
  {
    msg_.prc = std::move(arg);
    return Init_SBASSvData_reserved_3(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASSvData msg_;
};

class Init_SBASSvData_reserved_2
{
public:
  explicit Init_SBASSvData_reserved_2(::ublox_ubx_msgs::msg::SBASSvData & msg)
  : msg_(msg)
  {}
  Init_SBASSvData_prc reserved_2(::ublox_ubx_msgs::msg::SBASSvData::_reserved_2_type arg)
  {
    msg_.reserved_2 = std::move(arg);
    return Init_SBASSvData_prc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASSvData msg_;
};

class Init_SBASSvData_sv_service
{
public:
  explicit Init_SBASSvData_sv_service(::ublox_ubx_msgs::msg::SBASSvData & msg)
  : msg_(msg)
  {}
  Init_SBASSvData_reserved_2 sv_service(::ublox_ubx_msgs::msg::SBASSvData::_sv_service_type arg)
  {
    msg_.sv_service = std::move(arg);
    return Init_SBASSvData_reserved_2(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASSvData msg_;
};

class Init_SBASSvData_sv_sys
{
public:
  explicit Init_SBASSvData_sv_sys(::ublox_ubx_msgs::msg::SBASSvData & msg)
  : msg_(msg)
  {}
  Init_SBASSvData_sv_service sv_sys(::ublox_ubx_msgs::msg::SBASSvData::_sv_sys_type arg)
  {
    msg_.sv_sys = std::move(arg);
    return Init_SBASSvData_sv_service(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASSvData msg_;
};

class Init_SBASSvData_udre
{
public:
  explicit Init_SBASSvData_udre(::ublox_ubx_msgs::msg::SBASSvData & msg)
  : msg_(msg)
  {}
  Init_SBASSvData_sv_sys udre(::ublox_ubx_msgs::msg::SBASSvData::_udre_type arg)
  {
    msg_.udre = std::move(arg);
    return Init_SBASSvData_sv_sys(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASSvData msg_;
};

class Init_SBASSvData_reserved_1
{
public:
  explicit Init_SBASSvData_reserved_1(::ublox_ubx_msgs::msg::SBASSvData & msg)
  : msg_(msg)
  {}
  Init_SBASSvData_udre reserved_1(::ublox_ubx_msgs::msg::SBASSvData::_reserved_1_type arg)
  {
    msg_.reserved_1 = std::move(arg);
    return Init_SBASSvData_udre(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASSvData msg_;
};

class Init_SBASSvData_svid
{
public:
  Init_SBASSvData_svid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SBASSvData_reserved_1 svid(::ublox_ubx_msgs::msg::SBASSvData::_svid_type arg)
  {
    msg_.svid = std::move(arg);
    return Init_SBASSvData_reserved_1(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SBASSvData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::SBASSvData>()
{
  return ublox_ubx_msgs::msg::builder::Init_SBASSvData_svid();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__BUILDER_HPP_
