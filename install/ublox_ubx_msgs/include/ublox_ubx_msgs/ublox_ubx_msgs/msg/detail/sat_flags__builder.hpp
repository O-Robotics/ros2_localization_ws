// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/SatFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/sat_flags__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_SatFlags_clas_corr_used
{
public:
  explicit Init_SatFlags_clas_corr_used(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::SatFlags clas_corr_used(::ublox_ubx_msgs::msg::SatFlags::_clas_corr_used_type arg)
  {
    msg_.clas_corr_used = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_do_corr_used
{
public:
  explicit Init_SatFlags_do_corr_used(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_clas_corr_used do_corr_used(::ublox_ubx_msgs::msg::SatFlags::_do_corr_used_type arg)
  {
    msg_.do_corr_used = std::move(arg);
    return Init_SatFlags_clas_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_cr_corr_used
{
public:
  explicit Init_SatFlags_cr_corr_used(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_do_corr_used cr_corr_used(::ublox_ubx_msgs::msg::SatFlags::_cr_corr_used_type arg)
  {
    msg_.cr_corr_used = std::move(arg);
    return Init_SatFlags_do_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_pr_corr_used
{
public:
  explicit Init_SatFlags_pr_corr_used(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_cr_corr_used pr_corr_used(::ublox_ubx_msgs::msg::SatFlags::_pr_corr_used_type arg)
  {
    msg_.pr_corr_used = std::move(arg);
    return Init_SatFlags_cr_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_spartn_corr_used
{
public:
  explicit Init_SatFlags_spartn_corr_used(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_pr_corr_used spartn_corr_used(::ublox_ubx_msgs::msg::SatFlags::_spartn_corr_used_type arg)
  {
    msg_.spartn_corr_used = std::move(arg);
    return Init_SatFlags_pr_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_slas_corr_used
{
public:
  explicit Init_SatFlags_slas_corr_used(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_spartn_corr_used slas_corr_used(::ublox_ubx_msgs::msg::SatFlags::_slas_corr_used_type arg)
  {
    msg_.slas_corr_used = std::move(arg);
    return Init_SatFlags_spartn_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_rtcm_corr_used
{
public:
  explicit Init_SatFlags_rtcm_corr_used(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_slas_corr_used rtcm_corr_used(::ublox_ubx_msgs::msg::SatFlags::_rtcm_corr_used_type arg)
  {
    msg_.rtcm_corr_used = std::move(arg);
    return Init_SatFlags_slas_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_sbas_corr_used
{
public:
  explicit Init_SatFlags_sbas_corr_used(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_rtcm_corr_used sbas_corr_used(::ublox_ubx_msgs::msg::SatFlags::_sbas_corr_used_type arg)
  {
    msg_.sbas_corr_used = std::move(arg);
    return Init_SatFlags_rtcm_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_aop_avail
{
public:
  explicit Init_SatFlags_aop_avail(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_sbas_corr_used aop_avail(::ublox_ubx_msgs::msg::SatFlags::_aop_avail_type arg)
  {
    msg_.aop_avail = std::move(arg);
    return Init_SatFlags_sbas_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_ano_avail
{
public:
  explicit Init_SatFlags_ano_avail(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_aop_avail ano_avail(::ublox_ubx_msgs::msg::SatFlags::_ano_avail_type arg)
  {
    msg_.ano_avail = std::move(arg);
    return Init_SatFlags_aop_avail(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_alm_avail
{
public:
  explicit Init_SatFlags_alm_avail(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_ano_avail alm_avail(::ublox_ubx_msgs::msg::SatFlags::_alm_avail_type arg)
  {
    msg_.alm_avail = std::move(arg);
    return Init_SatFlags_ano_avail(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_eph_avail
{
public:
  explicit Init_SatFlags_eph_avail(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_alm_avail eph_avail(::ublox_ubx_msgs::msg::SatFlags::_eph_avail_type arg)
  {
    msg_.eph_avail = std::move(arg);
    return Init_SatFlags_alm_avail(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_orbit_source
{
public:
  explicit Init_SatFlags_orbit_source(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_eph_avail orbit_source(::ublox_ubx_msgs::msg::SatFlags::_orbit_source_type arg)
  {
    msg_.orbit_source = std::move(arg);
    return Init_SatFlags_eph_avail(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_smoothed
{
public:
  explicit Init_SatFlags_smoothed(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_orbit_source smoothed(::ublox_ubx_msgs::msg::SatFlags::_smoothed_type arg)
  {
    msg_.smoothed = std::move(arg);
    return Init_SatFlags_orbit_source(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_diff_corr
{
public:
  explicit Init_SatFlags_diff_corr(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_smoothed diff_corr(::ublox_ubx_msgs::msg::SatFlags::_diff_corr_type arg)
  {
    msg_.diff_corr = std::move(arg);
    return Init_SatFlags_smoothed(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_health
{
public:
  explicit Init_SatFlags_health(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_diff_corr health(::ublox_ubx_msgs::msg::SatFlags::_health_type arg)
  {
    msg_.health = std::move(arg);
    return Init_SatFlags_diff_corr(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_sv_used
{
public:
  explicit Init_SatFlags_sv_used(::ublox_ubx_msgs::msg::SatFlags & msg)
  : msg_(msg)
  {}
  Init_SatFlags_health sv_used(::ublox_ubx_msgs::msg::SatFlags::_sv_used_type arg)
  {
    msg_.sv_used = std::move(arg);
    return Init_SatFlags_health(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

class Init_SatFlags_quality_ind
{
public:
  Init_SatFlags_quality_ind()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SatFlags_sv_used quality_ind(::ublox_ubx_msgs::msg::SatFlags::_quality_ind_type arg)
  {
    msg_.quality_ind = std::move(arg);
    return Init_SatFlags_sv_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SatFlags msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::SatFlags>()
{
  return ublox_ubx_msgs::msg::builder::Init_SatFlags_quality_ind();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__BUILDER_HPP_
