// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPVT.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavPVT_mag_acc
{
public:
  explicit Init_UBXNavPVT_mag_acc(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavPVT mag_acc(::ublox_ubx_msgs::msg::UBXNavPVT::_mag_acc_type arg)
  {
    msg_.mag_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_mag_dec
{
public:
  explicit Init_UBXNavPVT_mag_dec(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_mag_acc mag_dec(::ublox_ubx_msgs::msg::UBXNavPVT::_mag_dec_type arg)
  {
    msg_.mag_dec = std::move(arg);
    return Init_UBXNavPVT_mag_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_head_veh
{
public:
  explicit Init_UBXNavPVT_head_veh(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_mag_dec head_veh(::ublox_ubx_msgs::msg::UBXNavPVT::_head_veh_type arg)
  {
    msg_.head_veh = std::move(arg);
    return Init_UBXNavPVT_mag_dec(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_invalid_llh
{
public:
  explicit Init_UBXNavPVT_invalid_llh(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_head_veh invalid_llh(::ublox_ubx_msgs::msg::UBXNavPVT::_invalid_llh_type arg)
  {
    msg_.invalid_llh = std::move(arg);
    return Init_UBXNavPVT_head_veh(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_p_dop
{
public:
  explicit Init_UBXNavPVT_p_dop(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_invalid_llh p_dop(::ublox_ubx_msgs::msg::UBXNavPVT::_p_dop_type arg)
  {
    msg_.p_dop = std::move(arg);
    return Init_UBXNavPVT_invalid_llh(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_head_acc
{
public:
  explicit Init_UBXNavPVT_head_acc(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_p_dop head_acc(::ublox_ubx_msgs::msg::UBXNavPVT::_head_acc_type arg)
  {
    msg_.head_acc = std::move(arg);
    return Init_UBXNavPVT_p_dop(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_s_acc
{
public:
  explicit Init_UBXNavPVT_s_acc(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_head_acc s_acc(::ublox_ubx_msgs::msg::UBXNavPVT::_s_acc_type arg)
  {
    msg_.s_acc = std::move(arg);
    return Init_UBXNavPVT_head_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_head_mot
{
public:
  explicit Init_UBXNavPVT_head_mot(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_s_acc head_mot(::ublox_ubx_msgs::msg::UBXNavPVT::_head_mot_type arg)
  {
    msg_.head_mot = std::move(arg);
    return Init_UBXNavPVT_s_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_g_speed
{
public:
  explicit Init_UBXNavPVT_g_speed(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_head_mot g_speed(::ublox_ubx_msgs::msg::UBXNavPVT::_g_speed_type arg)
  {
    msg_.g_speed = std::move(arg);
    return Init_UBXNavPVT_head_mot(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_vel_d
{
public:
  explicit Init_UBXNavPVT_vel_d(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_g_speed vel_d(::ublox_ubx_msgs::msg::UBXNavPVT::_vel_d_type arg)
  {
    msg_.vel_d = std::move(arg);
    return Init_UBXNavPVT_g_speed(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_vel_e
{
public:
  explicit Init_UBXNavPVT_vel_e(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_vel_d vel_e(::ublox_ubx_msgs::msg::UBXNavPVT::_vel_e_type arg)
  {
    msg_.vel_e = std::move(arg);
    return Init_UBXNavPVT_vel_d(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_vel_n
{
public:
  explicit Init_UBXNavPVT_vel_n(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_vel_e vel_n(::ublox_ubx_msgs::msg::UBXNavPVT::_vel_n_type arg)
  {
    msg_.vel_n = std::move(arg);
    return Init_UBXNavPVT_vel_e(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_v_acc
{
public:
  explicit Init_UBXNavPVT_v_acc(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_vel_n v_acc(::ublox_ubx_msgs::msg::UBXNavPVT::_v_acc_type arg)
  {
    msg_.v_acc = std::move(arg);
    return Init_UBXNavPVT_vel_n(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_h_acc
{
public:
  explicit Init_UBXNavPVT_h_acc(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_v_acc h_acc(::ublox_ubx_msgs::msg::UBXNavPVT::_h_acc_type arg)
  {
    msg_.h_acc = std::move(arg);
    return Init_UBXNavPVT_v_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_hmsl
{
public:
  explicit Init_UBXNavPVT_hmsl(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_h_acc hmsl(::ublox_ubx_msgs::msg::UBXNavPVT::_hmsl_type arg)
  {
    msg_.hmsl = std::move(arg);
    return Init_UBXNavPVT_h_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_height
{
public:
  explicit Init_UBXNavPVT_height(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_hmsl height(::ublox_ubx_msgs::msg::UBXNavPVT::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_UBXNavPVT_hmsl(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_lat
{
public:
  explicit Init_UBXNavPVT_lat(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_height lat(::ublox_ubx_msgs::msg::UBXNavPVT::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_UBXNavPVT_height(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_lon
{
public:
  explicit Init_UBXNavPVT_lon(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_lat lon(::ublox_ubx_msgs::msg::UBXNavPVT::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_UBXNavPVT_lat(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_num_sv
{
public:
  explicit Init_UBXNavPVT_num_sv(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_lon num_sv(::ublox_ubx_msgs::msg::UBXNavPVT::_num_sv_type arg)
  {
    msg_.num_sv = std::move(arg);
    return Init_UBXNavPVT_lon(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_confirmed_time
{
public:
  explicit Init_UBXNavPVT_confirmed_time(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_num_sv confirmed_time(::ublox_ubx_msgs::msg::UBXNavPVT::_confirmed_time_type arg)
  {
    msg_.confirmed_time = std::move(arg);
    return Init_UBXNavPVT_num_sv(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_confirmed_date
{
public:
  explicit Init_UBXNavPVT_confirmed_date(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_confirmed_time confirmed_date(::ublox_ubx_msgs::msg::UBXNavPVT::_confirmed_date_type arg)
  {
    msg_.confirmed_date = std::move(arg);
    return Init_UBXNavPVT_confirmed_time(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_confirmed_avail
{
public:
  explicit Init_UBXNavPVT_confirmed_avail(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_confirmed_date confirmed_avail(::ublox_ubx_msgs::msg::UBXNavPVT::_confirmed_avail_type arg)
  {
    msg_.confirmed_avail = std::move(arg);
    return Init_UBXNavPVT_confirmed_date(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_carr_soln
{
public:
  explicit Init_UBXNavPVT_carr_soln(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_confirmed_avail carr_soln(::ublox_ubx_msgs::msg::UBXNavPVT::_carr_soln_type arg)
  {
    msg_.carr_soln = std::move(arg);
    return Init_UBXNavPVT_confirmed_avail(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_head_veh_valid
{
public:
  explicit Init_UBXNavPVT_head_veh_valid(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_carr_soln head_veh_valid(::ublox_ubx_msgs::msg::UBXNavPVT::_head_veh_valid_type arg)
  {
    msg_.head_veh_valid = std::move(arg);
    return Init_UBXNavPVT_carr_soln(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_psm
{
public:
  explicit Init_UBXNavPVT_psm(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_head_veh_valid psm(::ublox_ubx_msgs::msg::UBXNavPVT::_psm_type arg)
  {
    msg_.psm = std::move(arg);
    return Init_UBXNavPVT_head_veh_valid(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_diff_soln
{
public:
  explicit Init_UBXNavPVT_diff_soln(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_psm diff_soln(::ublox_ubx_msgs::msg::UBXNavPVT::_diff_soln_type arg)
  {
    msg_.diff_soln = std::move(arg);
    return Init_UBXNavPVT_psm(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_gnss_fix_ok
{
public:
  explicit Init_UBXNavPVT_gnss_fix_ok(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_diff_soln gnss_fix_ok(::ublox_ubx_msgs::msg::UBXNavPVT::_gnss_fix_ok_type arg)
  {
    msg_.gnss_fix_ok = std::move(arg);
    return Init_UBXNavPVT_diff_soln(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_gps_fix
{
public:
  explicit Init_UBXNavPVT_gps_fix(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_gnss_fix_ok gps_fix(::ublox_ubx_msgs::msg::UBXNavPVT::_gps_fix_type arg)
  {
    msg_.gps_fix = std::move(arg);
    return Init_UBXNavPVT_gnss_fix_ok(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_nano
{
public:
  explicit Init_UBXNavPVT_nano(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_gps_fix nano(::ublox_ubx_msgs::msg::UBXNavPVT::_nano_type arg)
  {
    msg_.nano = std::move(arg);
    return Init_UBXNavPVT_gps_fix(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_t_acc
{
public:
  explicit Init_UBXNavPVT_t_acc(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_nano t_acc(::ublox_ubx_msgs::msg::UBXNavPVT::_t_acc_type arg)
  {
    msg_.t_acc = std::move(arg);
    return Init_UBXNavPVT_nano(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_valid_mag
{
public:
  explicit Init_UBXNavPVT_valid_mag(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_t_acc valid_mag(::ublox_ubx_msgs::msg::UBXNavPVT::_valid_mag_type arg)
  {
    msg_.valid_mag = std::move(arg);
    return Init_UBXNavPVT_t_acc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_fully_resolved
{
public:
  explicit Init_UBXNavPVT_fully_resolved(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_valid_mag fully_resolved(::ublox_ubx_msgs::msg::UBXNavPVT::_fully_resolved_type arg)
  {
    msg_.fully_resolved = std::move(arg);
    return Init_UBXNavPVT_valid_mag(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_valid_time
{
public:
  explicit Init_UBXNavPVT_valid_time(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_fully_resolved valid_time(::ublox_ubx_msgs::msg::UBXNavPVT::_valid_time_type arg)
  {
    msg_.valid_time = std::move(arg);
    return Init_UBXNavPVT_fully_resolved(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_valid_date
{
public:
  explicit Init_UBXNavPVT_valid_date(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_valid_time valid_date(::ublox_ubx_msgs::msg::UBXNavPVT::_valid_date_type arg)
  {
    msg_.valid_date = std::move(arg);
    return Init_UBXNavPVT_valid_time(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_sec
{
public:
  explicit Init_UBXNavPVT_sec(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_valid_date sec(::ublox_ubx_msgs::msg::UBXNavPVT::_sec_type arg)
  {
    msg_.sec = std::move(arg);
    return Init_UBXNavPVT_valid_date(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_min
{
public:
  explicit Init_UBXNavPVT_min(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_sec min(::ublox_ubx_msgs::msg::UBXNavPVT::_min_type arg)
  {
    msg_.min = std::move(arg);
    return Init_UBXNavPVT_sec(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_hour
{
public:
  explicit Init_UBXNavPVT_hour(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_min hour(::ublox_ubx_msgs::msg::UBXNavPVT::_hour_type arg)
  {
    msg_.hour = std::move(arg);
    return Init_UBXNavPVT_min(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_day
{
public:
  explicit Init_UBXNavPVT_day(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_hour day(::ublox_ubx_msgs::msg::UBXNavPVT::_day_type arg)
  {
    msg_.day = std::move(arg);
    return Init_UBXNavPVT_hour(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_month
{
public:
  explicit Init_UBXNavPVT_month(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_day month(::ublox_ubx_msgs::msg::UBXNavPVT::_month_type arg)
  {
    msg_.month = std::move(arg);
    return Init_UBXNavPVT_day(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_year
{
public:
  explicit Init_UBXNavPVT_year(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_month year(::ublox_ubx_msgs::msg::UBXNavPVT::_year_type arg)
  {
    msg_.year = std::move(arg);
    return Init_UBXNavPVT_month(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_itow
{
public:
  explicit Init_UBXNavPVT_itow(::ublox_ubx_msgs::msg::UBXNavPVT & msg)
  : msg_(msg)
  {}
  Init_UBXNavPVT_year itow(::ublox_ubx_msgs::msg::UBXNavPVT::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavPVT_year(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

class Init_UBXNavPVT_header
{
public:
  Init_UBXNavPVT_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavPVT_itow header(::ublox_ubx_msgs::msg::UBXNavPVT::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavPVT_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavPVT msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavPVT>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavPVT_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__BUILDER_HPP_
