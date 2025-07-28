// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/ESFSensorStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_ESFSensorStatus_fault_noisy_meas
{
public:
  explicit Init_ESFSensorStatus_fault_noisy_meas(::ublox_ubx_msgs::msg::ESFSensorStatus & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::ESFSensorStatus fault_noisy_meas(::ublox_ubx_msgs::msg::ESFSensorStatus::_fault_noisy_meas_type arg)
  {
    msg_.fault_noisy_meas = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

class Init_ESFSensorStatus_fault_missing_meas
{
public:
  explicit Init_ESFSensorStatus_fault_missing_meas(::ublox_ubx_msgs::msg::ESFSensorStatus & msg)
  : msg_(msg)
  {}
  Init_ESFSensorStatus_fault_noisy_meas fault_missing_meas(::ublox_ubx_msgs::msg::ESFSensorStatus::_fault_missing_meas_type arg)
  {
    msg_.fault_missing_meas = std::move(arg);
    return Init_ESFSensorStatus_fault_noisy_meas(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

class Init_ESFSensorStatus_fault_bad_ttag
{
public:
  explicit Init_ESFSensorStatus_fault_bad_ttag(::ublox_ubx_msgs::msg::ESFSensorStatus & msg)
  : msg_(msg)
  {}
  Init_ESFSensorStatus_fault_missing_meas fault_bad_ttag(::ublox_ubx_msgs::msg::ESFSensorStatus::_fault_bad_ttag_type arg)
  {
    msg_.fault_bad_ttag = std::move(arg);
    return Init_ESFSensorStatus_fault_missing_meas(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

class Init_ESFSensorStatus_fault_bad_meas
{
public:
  explicit Init_ESFSensorStatus_fault_bad_meas(::ublox_ubx_msgs::msg::ESFSensorStatus & msg)
  : msg_(msg)
  {}
  Init_ESFSensorStatus_fault_bad_ttag fault_bad_meas(::ublox_ubx_msgs::msg::ESFSensorStatus::_fault_bad_meas_type arg)
  {
    msg_.fault_bad_meas = std::move(arg);
    return Init_ESFSensorStatus_fault_bad_ttag(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

class Init_ESFSensorStatus_freq
{
public:
  explicit Init_ESFSensorStatus_freq(::ublox_ubx_msgs::msg::ESFSensorStatus & msg)
  : msg_(msg)
  {}
  Init_ESFSensorStatus_fault_bad_meas freq(::ublox_ubx_msgs::msg::ESFSensorStatus::_freq_type arg)
  {
    msg_.freq = std::move(arg);
    return Init_ESFSensorStatus_fault_bad_meas(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

class Init_ESFSensorStatus_time_status
{
public:
  explicit Init_ESFSensorStatus_time_status(::ublox_ubx_msgs::msg::ESFSensorStatus & msg)
  : msg_(msg)
  {}
  Init_ESFSensorStatus_freq time_status(::ublox_ubx_msgs::msg::ESFSensorStatus::_time_status_type arg)
  {
    msg_.time_status = std::move(arg);
    return Init_ESFSensorStatus_freq(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

class Init_ESFSensorStatus_calib_status
{
public:
  explicit Init_ESFSensorStatus_calib_status(::ublox_ubx_msgs::msg::ESFSensorStatus & msg)
  : msg_(msg)
  {}
  Init_ESFSensorStatus_time_status calib_status(::ublox_ubx_msgs::msg::ESFSensorStatus::_calib_status_type arg)
  {
    msg_.calib_status = std::move(arg);
    return Init_ESFSensorStatus_time_status(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

class Init_ESFSensorStatus_ready
{
public:
  explicit Init_ESFSensorStatus_ready(::ublox_ubx_msgs::msg::ESFSensorStatus & msg)
  : msg_(msg)
  {}
  Init_ESFSensorStatus_calib_status ready(::ublox_ubx_msgs::msg::ESFSensorStatus::_ready_type arg)
  {
    msg_.ready = std::move(arg);
    return Init_ESFSensorStatus_calib_status(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

class Init_ESFSensorStatus_used
{
public:
  explicit Init_ESFSensorStatus_used(::ublox_ubx_msgs::msg::ESFSensorStatus & msg)
  : msg_(msg)
  {}
  Init_ESFSensorStatus_ready used(::ublox_ubx_msgs::msg::ESFSensorStatus::_used_type arg)
  {
    msg_.used = std::move(arg);
    return Init_ESFSensorStatus_ready(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

class Init_ESFSensorStatus_sensor_data_type
{
public:
  Init_ESFSensorStatus_sensor_data_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESFSensorStatus_used sensor_data_type(::ublox_ubx_msgs::msg::ESFSensorStatus::_sensor_data_type_type arg)
  {
    msg_.sensor_data_type = std::move(arg);
    return Init_ESFSensorStatus_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFSensorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::ESFSensorStatus>()
{
  return ublox_ubx_msgs::msg::builder::Init_ESFSensorStatus_sensor_data_type();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__BUILDER_HPP_
