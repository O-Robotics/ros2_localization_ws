// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/TrkStat.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/trk_stat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_TrkStat_sub_half_cyc
{
public:
  explicit Init_TrkStat_sub_half_cyc(::ublox_ubx_msgs::msg::TrkStat & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::TrkStat sub_half_cyc(::ublox_ubx_msgs::msg::TrkStat::_sub_half_cyc_type arg)
  {
    msg_.sub_half_cyc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::TrkStat msg_;
};

class Init_TrkStat_half_cyc
{
public:
  explicit Init_TrkStat_half_cyc(::ublox_ubx_msgs::msg::TrkStat & msg)
  : msg_(msg)
  {}
  Init_TrkStat_sub_half_cyc half_cyc(::ublox_ubx_msgs::msg::TrkStat::_half_cyc_type arg)
  {
    msg_.half_cyc = std::move(arg);
    return Init_TrkStat_sub_half_cyc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::TrkStat msg_;
};

class Init_TrkStat_cp_valid
{
public:
  explicit Init_TrkStat_cp_valid(::ublox_ubx_msgs::msg::TrkStat & msg)
  : msg_(msg)
  {}
  Init_TrkStat_half_cyc cp_valid(::ublox_ubx_msgs::msg::TrkStat::_cp_valid_type arg)
  {
    msg_.cp_valid = std::move(arg);
    return Init_TrkStat_half_cyc(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::TrkStat msg_;
};

class Init_TrkStat_pr_valid
{
public:
  Init_TrkStat_pr_valid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrkStat_cp_valid pr_valid(::ublox_ubx_msgs::msg::TrkStat::_pr_valid_type arg)
  {
    msg_.pr_valid = std::move(arg);
    return Init_TrkStat_cp_valid(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::TrkStat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::TrkStat>()
{
  return ublox_ubx_msgs::msg::builder::Init_TrkStat_pr_valid();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__BUILDER_HPP_
