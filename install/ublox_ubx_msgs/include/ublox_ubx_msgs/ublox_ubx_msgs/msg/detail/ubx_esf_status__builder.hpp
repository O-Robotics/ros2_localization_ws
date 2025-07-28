// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXEsfStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_esf_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXEsfStatus_sensor_statuses
{
public:
  explicit Init_UBXEsfStatus_sensor_statuses(::ublox_ubx_msgs::msg::UBXEsfStatus & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXEsfStatus sensor_statuses(::ublox_ubx_msgs::msg::UBXEsfStatus::_sensor_statuses_type arg)
  {
    msg_.sensor_statuses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

class Init_UBXEsfStatus_num_sens
{
public:
  explicit Init_UBXEsfStatus_num_sens(::ublox_ubx_msgs::msg::UBXEsfStatus & msg)
  : msg_(msg)
  {}
  Init_UBXEsfStatus_sensor_statuses num_sens(::ublox_ubx_msgs::msg::UBXEsfStatus::_num_sens_type arg)
  {
    msg_.num_sens = std::move(arg);
    return Init_UBXEsfStatus_sensor_statuses(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

class Init_UBXEsfStatus_fusion_mode
{
public:
  explicit Init_UBXEsfStatus_fusion_mode(::ublox_ubx_msgs::msg::UBXEsfStatus & msg)
  : msg_(msg)
  {}
  Init_UBXEsfStatus_num_sens fusion_mode(::ublox_ubx_msgs::msg::UBXEsfStatus::_fusion_mode_type arg)
  {
    msg_.fusion_mode = std::move(arg);
    return Init_UBXEsfStatus_num_sens(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

class Init_UBXEsfStatus_imu_init_status
{
public:
  explicit Init_UBXEsfStatus_imu_init_status(::ublox_ubx_msgs::msg::UBXEsfStatus & msg)
  : msg_(msg)
  {}
  Init_UBXEsfStatus_fusion_mode imu_init_status(::ublox_ubx_msgs::msg::UBXEsfStatus::_imu_init_status_type arg)
  {
    msg_.imu_init_status = std::move(arg);
    return Init_UBXEsfStatus_fusion_mode(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

class Init_UBXEsfStatus_ins_init_status
{
public:
  explicit Init_UBXEsfStatus_ins_init_status(::ublox_ubx_msgs::msg::UBXEsfStatus & msg)
  : msg_(msg)
  {}
  Init_UBXEsfStatus_imu_init_status ins_init_status(::ublox_ubx_msgs::msg::UBXEsfStatus::_ins_init_status_type arg)
  {
    msg_.ins_init_status = std::move(arg);
    return Init_UBXEsfStatus_imu_init_status(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

class Init_UBXEsfStatus_mnt_alg_status
{
public:
  explicit Init_UBXEsfStatus_mnt_alg_status(::ublox_ubx_msgs::msg::UBXEsfStatus & msg)
  : msg_(msg)
  {}
  Init_UBXEsfStatus_ins_init_status mnt_alg_status(::ublox_ubx_msgs::msg::UBXEsfStatus::_mnt_alg_status_type arg)
  {
    msg_.mnt_alg_status = std::move(arg);
    return Init_UBXEsfStatus_ins_init_status(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

class Init_UBXEsfStatus_wt_init_status
{
public:
  explicit Init_UBXEsfStatus_wt_init_status(::ublox_ubx_msgs::msg::UBXEsfStatus & msg)
  : msg_(msg)
  {}
  Init_UBXEsfStatus_mnt_alg_status wt_init_status(::ublox_ubx_msgs::msg::UBXEsfStatus::_wt_init_status_type arg)
  {
    msg_.wt_init_status = std::move(arg);
    return Init_UBXEsfStatus_mnt_alg_status(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

class Init_UBXEsfStatus_version
{
public:
  explicit Init_UBXEsfStatus_version(::ublox_ubx_msgs::msg::UBXEsfStatus & msg)
  : msg_(msg)
  {}
  Init_UBXEsfStatus_wt_init_status version(::ublox_ubx_msgs::msg::UBXEsfStatus::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXEsfStatus_wt_init_status(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

class Init_UBXEsfStatus_itow
{
public:
  explicit Init_UBXEsfStatus_itow(::ublox_ubx_msgs::msg::UBXEsfStatus & msg)
  : msg_(msg)
  {}
  Init_UBXEsfStatus_version itow(::ublox_ubx_msgs::msg::UBXEsfStatus::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXEsfStatus_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

class Init_UBXEsfStatus_header
{
public:
  Init_UBXEsfStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXEsfStatus_itow header(::ublox_ubx_msgs::msg::UBXEsfStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXEsfStatus_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXEsfStatus>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXEsfStatus_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__BUILDER_HPP_
