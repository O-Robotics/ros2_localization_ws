// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmMeasx.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_rxm_measx__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXRxmMeasx_sv_data
{
public:
  explicit Init_UBXRxmMeasx_sv_data(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXRxmMeasx sv_data(::ublox_ubx_msgs::msg::UBXRxmMeasx::_sv_data_type arg)
  {
    msg_.sv_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_flags
{
public:
  explicit Init_UBXRxmMeasx_flags(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_sv_data flags(::ublox_ubx_msgs::msg::UBXRxmMeasx::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_UBXRxmMeasx_sv_data(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_num_sv
{
public:
  explicit Init_UBXRxmMeasx_num_sv(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_flags num_sv(::ublox_ubx_msgs::msg::UBXRxmMeasx::_num_sv_type arg)
  {
    msg_.num_sv = std::move(arg);
    return Init_UBXRxmMeasx_flags(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_qzss_tow_acc
{
public:
  explicit Init_UBXRxmMeasx_qzss_tow_acc(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_num_sv qzss_tow_acc(::ublox_ubx_msgs::msg::UBXRxmMeasx::_qzss_tow_acc_type arg)
  {
    msg_.qzss_tow_acc = std::move(arg);
    return Init_UBXRxmMeasx_num_sv(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_bds_tow_acc
{
public:
  explicit Init_UBXRxmMeasx_bds_tow_acc(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_qzss_tow_acc bds_tow_acc(::ublox_ubx_msgs::msg::UBXRxmMeasx::_bds_tow_acc_type arg)
  {
    msg_.bds_tow_acc = std::move(arg);
    return Init_UBXRxmMeasx_qzss_tow_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_glo_tow_acc
{
public:
  explicit Init_UBXRxmMeasx_glo_tow_acc(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_bds_tow_acc glo_tow_acc(::ublox_ubx_msgs::msg::UBXRxmMeasx::_glo_tow_acc_type arg)
  {
    msg_.glo_tow_acc = std::move(arg);
    return Init_UBXRxmMeasx_bds_tow_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_gps_tow_acc
{
public:
  explicit Init_UBXRxmMeasx_gps_tow_acc(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_glo_tow_acc gps_tow_acc(::ublox_ubx_msgs::msg::UBXRxmMeasx::_gps_tow_acc_type arg)
  {
    msg_.gps_tow_acc = std::move(arg);
    return Init_UBXRxmMeasx_glo_tow_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_qzss_tow
{
public:
  explicit Init_UBXRxmMeasx_qzss_tow(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_gps_tow_acc qzss_tow(::ublox_ubx_msgs::msg::UBXRxmMeasx::_qzss_tow_type arg)
  {
    msg_.qzss_tow = std::move(arg);
    return Init_UBXRxmMeasx_gps_tow_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_bds_tow
{
public:
  explicit Init_UBXRxmMeasx_bds_tow(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_qzss_tow bds_tow(::ublox_ubx_msgs::msg::UBXRxmMeasx::_bds_tow_type arg)
  {
    msg_.bds_tow = std::move(arg);
    return Init_UBXRxmMeasx_qzss_tow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_glo_tow
{
public:
  explicit Init_UBXRxmMeasx_glo_tow(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_bds_tow glo_tow(::ublox_ubx_msgs::msg::UBXRxmMeasx::_glo_tow_type arg)
  {
    msg_.glo_tow = std::move(arg);
    return Init_UBXRxmMeasx_bds_tow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_gps_tow
{
public:
  explicit Init_UBXRxmMeasx_gps_tow(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_glo_tow gps_tow(::ublox_ubx_msgs::msg::UBXRxmMeasx::_gps_tow_type arg)
  {
    msg_.gps_tow = std::move(arg);
    return Init_UBXRxmMeasx_glo_tow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_version
{
public:
  explicit Init_UBXRxmMeasx_version(::ublox_ubx_msgs::msg::UBXRxmMeasx & msg)
  : msg_(msg)
  {}
  Init_UBXRxmMeasx_gps_tow version(::ublox_ubx_msgs::msg::UBXRxmMeasx::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXRxmMeasx_gps_tow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

class Init_UBXRxmMeasx_header
{
public:
  Init_UBXRxmMeasx_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXRxmMeasx_version header(::ublox_ubx_msgs::msg::UBXRxmMeasx::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXRxmMeasx_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXRxmMeasx msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXRxmMeasx>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXRxmMeasx_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__BUILDER_HPP_
