// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavRelPosNED.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavRelPosNED_rel_pos_normalized
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_normalized(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED rel_pos_normalized(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_normalized_type arg)
  {
    msg_.rel_pos_normalized = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_heading_valid
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_heading_valid(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_normalized rel_pos_heading_valid(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_heading_valid_type arg)
  {
    msg_.rel_pos_heading_valid = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_normalized(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_ref_obs_miss
{
public:
  explicit Init_UBXNavRelPosNED_ref_obs_miss(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_heading_valid ref_obs_miss(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_ref_obs_miss_type arg)
  {
    msg_.ref_obs_miss = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_heading_valid(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_ref_pos_miss
{
public:
  explicit Init_UBXNavRelPosNED_ref_pos_miss(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_ref_obs_miss ref_pos_miss(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_ref_pos_miss_type arg)
  {
    msg_.ref_pos_miss = std::move(arg);
    return Init_UBXNavRelPosNED_ref_obs_miss(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_is_moving
{
public:
  explicit Init_UBXNavRelPosNED_is_moving(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_ref_pos_miss is_moving(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_is_moving_type arg)
  {
    msg_.is_moving = std::move(arg);
    return Init_UBXNavRelPosNED_ref_pos_miss(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_carr_soln
{
public:
  explicit Init_UBXNavRelPosNED_carr_soln(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_is_moving carr_soln(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_carr_soln_type arg)
  {
    msg_.carr_soln = std::move(arg);
    return Init_UBXNavRelPosNED_is_moving(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_valid
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_valid(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_carr_soln rel_pos_valid(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_valid_type arg)
  {
    msg_.rel_pos_valid = std::move(arg);
    return Init_UBXNavRelPosNED_carr_soln(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_diff_soln
{
public:
  explicit Init_UBXNavRelPosNED_diff_soln(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_valid diff_soln(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_diff_soln_type arg)
  {
    msg_.diff_soln = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_valid(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_gnss_fix_ok
{
public:
  explicit Init_UBXNavRelPosNED_gnss_fix_ok(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_diff_soln gnss_fix_ok(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_gnss_fix_ok_type arg)
  {
    msg_.gnss_fix_ok = std::move(arg);
    return Init_UBXNavRelPosNED_diff_soln(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_acc_heading
{
public:
  explicit Init_UBXNavRelPosNED_acc_heading(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_gnss_fix_ok acc_heading(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_acc_heading_type arg)
  {
    msg_.acc_heading = std::move(arg);
    return Init_UBXNavRelPosNED_gnss_fix_ok(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_acc_length
{
public:
  explicit Init_UBXNavRelPosNED_acc_length(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_acc_heading acc_length(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_acc_length_type arg)
  {
    msg_.acc_length = std::move(arg);
    return Init_UBXNavRelPosNED_acc_heading(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_acc_d
{
public:
  explicit Init_UBXNavRelPosNED_acc_d(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_acc_length acc_d(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_acc_d_type arg)
  {
    msg_.acc_d = std::move(arg);
    return Init_UBXNavRelPosNED_acc_length(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_acc_e
{
public:
  explicit Init_UBXNavRelPosNED_acc_e(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_acc_d acc_e(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_acc_e_type arg)
  {
    msg_.acc_e = std::move(arg);
    return Init_UBXNavRelPosNED_acc_d(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_acc_n
{
public:
  explicit Init_UBXNavRelPosNED_acc_n(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_acc_e acc_n(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_acc_n_type arg)
  {
    msg_.acc_n = std::move(arg);
    return Init_UBXNavRelPosNED_acc_e(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_hp_length
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_hp_length(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_acc_n rel_pos_hp_length(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_hp_length_type arg)
  {
    msg_.rel_pos_hp_length = std::move(arg);
    return Init_UBXNavRelPosNED_acc_n(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_hp_d
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_hp_d(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_hp_length rel_pos_hp_d(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_hp_d_type arg)
  {
    msg_.rel_pos_hp_d = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_hp_length(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_hp_e
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_hp_e(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_hp_d rel_pos_hp_e(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_hp_e_type arg)
  {
    msg_.rel_pos_hp_e = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_hp_d(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_hp_n
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_hp_n(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_hp_e rel_pos_hp_n(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_hp_n_type arg)
  {
    msg_.rel_pos_hp_n = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_hp_e(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_heading
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_heading(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_hp_n rel_pos_heading(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_heading_type arg)
  {
    msg_.rel_pos_heading = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_hp_n(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_length
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_length(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_heading rel_pos_length(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_length_type arg)
  {
    msg_.rel_pos_length = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_heading(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_d
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_d(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_length rel_pos_d(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_d_type arg)
  {
    msg_.rel_pos_d = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_length(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_e
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_e(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_d rel_pos_e(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_e_type arg)
  {
    msg_.rel_pos_e = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_d(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_rel_pos_n
{
public:
  explicit Init_UBXNavRelPosNED_rel_pos_n(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_e rel_pos_n(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_rel_pos_n_type arg)
  {
    msg_.rel_pos_n = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_e(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_itow
{
public:
  explicit Init_UBXNavRelPosNED_itow(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_rel_pos_n itow(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavRelPosNED_rel_pos_n(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_ref_station_id
{
public:
  explicit Init_UBXNavRelPosNED_ref_station_id(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_itow ref_station_id(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_ref_station_id_type arg)
  {
    msg_.ref_station_id = std::move(arg);
    return Init_UBXNavRelPosNED_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_version
{
public:
  explicit Init_UBXNavRelPosNED_version(::ublox_ubx_msgs::msg::UBXNavRelPosNED & msg)
  : msg_(msg)
  {}
  Init_UBXNavRelPosNED_ref_station_id version(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXNavRelPosNED_ref_station_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

class Init_UBXNavRelPosNED_header
{
public:
  Init_UBXNavRelPosNED_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavRelPosNED_version header(::ublox_ubx_msgs::msg::UBXNavRelPosNED::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavRelPosNED_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavRelPosNED msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavRelPosNED>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavRelPosNED_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__BUILDER_HPP_
