// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/MeasxData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/measx_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_MeasxData_pseu_range_rms_err
{
public:
  explicit Init_MeasxData_pseu_range_rms_err(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::MeasxData pseu_range_rms_err(::ublox_ubx_msgs::msg::MeasxData::_pseu_range_rms_err_type arg)
  {
    msg_.pseu_range_rms_err = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_int_code_phase
{
public:
  explicit Init_MeasxData_int_code_phase(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  Init_MeasxData_pseu_range_rms_err int_code_phase(::ublox_ubx_msgs::msg::MeasxData::_int_code_phase_type arg)
  {
    msg_.int_code_phase = std::move(arg);
    return Init_MeasxData_pseu_range_rms_err(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_code_phase
{
public:
  explicit Init_MeasxData_code_phase(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  Init_MeasxData_int_code_phase code_phase(::ublox_ubx_msgs::msg::MeasxData::_code_phase_type arg)
  {
    msg_.code_phase = std::move(arg);
    return Init_MeasxData_int_code_phase(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_frac_chips
{
public:
  explicit Init_MeasxData_frac_chips(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  Init_MeasxData_code_phase frac_chips(::ublox_ubx_msgs::msg::MeasxData::_frac_chips_type arg)
  {
    msg_.frac_chips = std::move(arg);
    return Init_MeasxData_code_phase(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_whole_chips
{
public:
  explicit Init_MeasxData_whole_chips(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  Init_MeasxData_frac_chips whole_chips(::ublox_ubx_msgs::msg::MeasxData::_whole_chips_type arg)
  {
    msg_.whole_chips = std::move(arg);
    return Init_MeasxData_frac_chips(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_doppler_hz
{
public:
  explicit Init_MeasxData_doppler_hz(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  Init_MeasxData_whole_chips doppler_hz(::ublox_ubx_msgs::msg::MeasxData::_doppler_hz_type arg)
  {
    msg_.doppler_hz = std::move(arg);
    return Init_MeasxData_whole_chips(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_doppler_ms
{
public:
  explicit Init_MeasxData_doppler_ms(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  Init_MeasxData_doppler_hz doppler_ms(::ublox_ubx_msgs::msg::MeasxData::_doppler_ms_type arg)
  {
    msg_.doppler_ms = std::move(arg);
    return Init_MeasxData_doppler_hz(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_mpath_indic
{
public:
  explicit Init_MeasxData_mpath_indic(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  Init_MeasxData_doppler_ms mpath_indic(::ublox_ubx_msgs::msg::MeasxData::_mpath_indic_type arg)
  {
    msg_.mpath_indic = std::move(arg);
    return Init_MeasxData_doppler_ms(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_c_no
{
public:
  explicit Init_MeasxData_c_no(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  Init_MeasxData_mpath_indic c_no(::ublox_ubx_msgs::msg::MeasxData::_c_no_type arg)
  {
    msg_.c_no = std::move(arg);
    return Init_MeasxData_mpath_indic(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_sv_id
{
public:
  explicit Init_MeasxData_sv_id(::ublox_ubx_msgs::msg::MeasxData & msg)
  : msg_(msg)
  {}
  Init_MeasxData_c_no sv_id(::ublox_ubx_msgs::msg::MeasxData::_sv_id_type arg)
  {
    msg_.sv_id = std::move(arg);
    return Init_MeasxData_c_no(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

class Init_MeasxData_gnss_id
{
public:
  Init_MeasxData_gnss_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeasxData_sv_id gnss_id(::ublox_ubx_msgs::msg::MeasxData::_gnss_id_type arg)
  {
    msg_.gnss_id = std::move(arg);
    return Init_MeasxData_sv_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::MeasxData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::MeasxData>()
{
  return ublox_ubx_msgs::msg::builder::Init_MeasxData_gnss_id();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__BUILDER_HPP_
