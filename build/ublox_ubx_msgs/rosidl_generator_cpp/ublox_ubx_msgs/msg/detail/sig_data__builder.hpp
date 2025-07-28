// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/SigData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/sig_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_SigData_sig_flags
{
public:
  explicit Init_SigData_sig_flags(::ublox_ubx_msgs::msg::SigData & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::SigData sig_flags(::ublox_ubx_msgs::msg::SigData::_sig_flags_type arg)
  {
    msg_.sig_flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

class Init_SigData_iono_model
{
public:
  explicit Init_SigData_iono_model(::ublox_ubx_msgs::msg::SigData & msg)
  : msg_(msg)
  {}
  Init_SigData_sig_flags iono_model(::ublox_ubx_msgs::msg::SigData::_iono_model_type arg)
  {
    msg_.iono_model = std::move(arg);
    return Init_SigData_sig_flags(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

class Init_SigData_corr_source
{
public:
  explicit Init_SigData_corr_source(::ublox_ubx_msgs::msg::SigData & msg)
  : msg_(msg)
  {}
  Init_SigData_iono_model corr_source(::ublox_ubx_msgs::msg::SigData::_corr_source_type arg)
  {
    msg_.corr_source = std::move(arg);
    return Init_SigData_iono_model(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

class Init_SigData_quality_ind
{
public:
  explicit Init_SigData_quality_ind(::ublox_ubx_msgs::msg::SigData & msg)
  : msg_(msg)
  {}
  Init_SigData_corr_source quality_ind(::ublox_ubx_msgs::msg::SigData::_quality_ind_type arg)
  {
    msg_.quality_ind = std::move(arg);
    return Init_SigData_corr_source(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

class Init_SigData_cno
{
public:
  explicit Init_SigData_cno(::ublox_ubx_msgs::msg::SigData & msg)
  : msg_(msg)
  {}
  Init_SigData_quality_ind cno(::ublox_ubx_msgs::msg::SigData::_cno_type arg)
  {
    msg_.cno = std::move(arg);
    return Init_SigData_quality_ind(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

class Init_SigData_pr_res
{
public:
  explicit Init_SigData_pr_res(::ublox_ubx_msgs::msg::SigData & msg)
  : msg_(msg)
  {}
  Init_SigData_cno pr_res(::ublox_ubx_msgs::msg::SigData::_pr_res_type arg)
  {
    msg_.pr_res = std::move(arg);
    return Init_SigData_cno(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

class Init_SigData_freq_id
{
public:
  explicit Init_SigData_freq_id(::ublox_ubx_msgs::msg::SigData & msg)
  : msg_(msg)
  {}
  Init_SigData_pr_res freq_id(::ublox_ubx_msgs::msg::SigData::_freq_id_type arg)
  {
    msg_.freq_id = std::move(arg);
    return Init_SigData_pr_res(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

class Init_SigData_sig_id
{
public:
  explicit Init_SigData_sig_id(::ublox_ubx_msgs::msg::SigData & msg)
  : msg_(msg)
  {}
  Init_SigData_freq_id sig_id(::ublox_ubx_msgs::msg::SigData::_sig_id_type arg)
  {
    msg_.sig_id = std::move(arg);
    return Init_SigData_freq_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

class Init_SigData_sv_id
{
public:
  explicit Init_SigData_sv_id(::ublox_ubx_msgs::msg::SigData & msg)
  : msg_(msg)
  {}
  Init_SigData_sig_id sv_id(::ublox_ubx_msgs::msg::SigData::_sv_id_type arg)
  {
    msg_.sv_id = std::move(arg);
    return Init_SigData_sig_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

class Init_SigData_gnss_id
{
public:
  Init_SigData_gnss_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SigData_sv_id gnss_id(::ublox_ubx_msgs::msg::SigData::_gnss_id_type arg)
  {
    msg_.gnss_id = std::move(arg);
    return Init_SigData_sv_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::SigData>()
{
  return ublox_ubx_msgs::msg::builder::Init_SigData_gnss_id();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__BUILDER_HPP_
