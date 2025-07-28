// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavStatus_msss
{
public:
  explicit Init_UBXNavStatus_msss(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavStatus msss(::ublox_ubx_msgs::msg::UBXNavStatus::_msss_type arg)
  {
    msg_.msss = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_ttff
{
public:
  explicit Init_UBXNavStatus_ttff(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_msss ttff(::ublox_ubx_msgs::msg::UBXNavStatus::_ttff_type arg)
  {
    msg_.ttff = std::move(arg);
    return Init_UBXNavStatus_msss(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_carr_soln
{
public:
  explicit Init_UBXNavStatus_carr_soln(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_ttff carr_soln(::ublox_ubx_msgs::msg::UBXNavStatus::_carr_soln_type arg)
  {
    msg_.carr_soln = std::move(arg);
    return Init_UBXNavStatus_ttff(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_spoof_det
{
public:
  explicit Init_UBXNavStatus_spoof_det(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_carr_soln spoof_det(::ublox_ubx_msgs::msg::UBXNavStatus::_spoof_det_type arg)
  {
    msg_.spoof_det = std::move(arg);
    return Init_UBXNavStatus_carr_soln(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_psm
{
public:
  explicit Init_UBXNavStatus_psm(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_spoof_det psm(::ublox_ubx_msgs::msg::UBXNavStatus::_psm_type arg)
  {
    msg_.psm = std::move(arg);
    return Init_UBXNavStatus_spoof_det(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_map_matching
{
public:
  explicit Init_UBXNavStatus_map_matching(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_psm map_matching(::ublox_ubx_msgs::msg::UBXNavStatus::_map_matching_type arg)
  {
    msg_.map_matching = std::move(arg);
    return Init_UBXNavStatus_psm(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_carr_soln_valid
{
public:
  explicit Init_UBXNavStatus_carr_soln_valid(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_map_matching carr_soln_valid(::ublox_ubx_msgs::msg::UBXNavStatus::_carr_soln_valid_type arg)
  {
    msg_.carr_soln_valid = std::move(arg);
    return Init_UBXNavStatus_map_matching(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_diff_corr
{
public:
  explicit Init_UBXNavStatus_diff_corr(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_carr_soln_valid diff_corr(::ublox_ubx_msgs::msg::UBXNavStatus::_diff_corr_type arg)
  {
    msg_.diff_corr = std::move(arg);
    return Init_UBXNavStatus_carr_soln_valid(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_tow_set
{
public:
  explicit Init_UBXNavStatus_tow_set(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_diff_corr tow_set(::ublox_ubx_msgs::msg::UBXNavStatus::_tow_set_type arg)
  {
    msg_.tow_set = std::move(arg);
    return Init_UBXNavStatus_diff_corr(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_wkn_set
{
public:
  explicit Init_UBXNavStatus_wkn_set(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_tow_set wkn_set(::ublox_ubx_msgs::msg::UBXNavStatus::_wkn_set_type arg)
  {
    msg_.wkn_set = std::move(arg);
    return Init_UBXNavStatus_tow_set(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_diff_soln
{
public:
  explicit Init_UBXNavStatus_diff_soln(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_wkn_set diff_soln(::ublox_ubx_msgs::msg::UBXNavStatus::_diff_soln_type arg)
  {
    msg_.diff_soln = std::move(arg);
    return Init_UBXNavStatus_wkn_set(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_gps_fix_ok
{
public:
  explicit Init_UBXNavStatus_gps_fix_ok(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_diff_soln gps_fix_ok(::ublox_ubx_msgs::msg::UBXNavStatus::_gps_fix_ok_type arg)
  {
    msg_.gps_fix_ok = std::move(arg);
    return Init_UBXNavStatus_diff_soln(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_gps_fix
{
public:
  explicit Init_UBXNavStatus_gps_fix(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_gps_fix_ok gps_fix(::ublox_ubx_msgs::msg::UBXNavStatus::_gps_fix_type arg)
  {
    msg_.gps_fix = std::move(arg);
    return Init_UBXNavStatus_gps_fix_ok(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_itow
{
public:
  explicit Init_UBXNavStatus_itow(::ublox_ubx_msgs::msg::UBXNavStatus & msg)
  : msg_(msg)
  {}
  Init_UBXNavStatus_gps_fix itow(::ublox_ubx_msgs::msg::UBXNavStatus::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavStatus_gps_fix(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

class Init_UBXNavStatus_header
{
public:
  Init_UBXNavStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavStatus_itow header(::ublox_ubx_msgs::msg::UBXNavStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavStatus_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavStatus>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavStatus_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__BUILDER_HPP_
