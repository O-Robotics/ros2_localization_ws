// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/SigFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/sig_flags__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_SigFlags_do_corr_used
{
public:
  explicit Init_SigFlags_do_corr_used(::ublox_ubx_msgs::msg::SigFlags & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::SigFlags do_corr_used(::ublox_ubx_msgs::msg::SigFlags::_do_corr_used_type arg)
  {
    msg_.do_corr_used = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigFlags msg_;
};

class Init_SigFlags_cr_corr_used
{
public:
  explicit Init_SigFlags_cr_corr_used(::ublox_ubx_msgs::msg::SigFlags & msg)
  : msg_(msg)
  {}
  Init_SigFlags_do_corr_used cr_corr_used(::ublox_ubx_msgs::msg::SigFlags::_cr_corr_used_type arg)
  {
    msg_.cr_corr_used = std::move(arg);
    return Init_SigFlags_do_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigFlags msg_;
};

class Init_SigFlags_pr_corr_used
{
public:
  explicit Init_SigFlags_pr_corr_used(::ublox_ubx_msgs::msg::SigFlags & msg)
  : msg_(msg)
  {}
  Init_SigFlags_cr_corr_used pr_corr_used(::ublox_ubx_msgs::msg::SigFlags::_pr_corr_used_type arg)
  {
    msg_.pr_corr_used = std::move(arg);
    return Init_SigFlags_cr_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigFlags msg_;
};

class Init_SigFlags_do_used
{
public:
  explicit Init_SigFlags_do_used(::ublox_ubx_msgs::msg::SigFlags & msg)
  : msg_(msg)
  {}
  Init_SigFlags_pr_corr_used do_used(::ublox_ubx_msgs::msg::SigFlags::_do_used_type arg)
  {
    msg_.do_used = std::move(arg);
    return Init_SigFlags_pr_corr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigFlags msg_;
};

class Init_SigFlags_cr_used
{
public:
  explicit Init_SigFlags_cr_used(::ublox_ubx_msgs::msg::SigFlags & msg)
  : msg_(msg)
  {}
  Init_SigFlags_do_used cr_used(::ublox_ubx_msgs::msg::SigFlags::_cr_used_type arg)
  {
    msg_.cr_used = std::move(arg);
    return Init_SigFlags_do_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigFlags msg_;
};

class Init_SigFlags_pr_used
{
public:
  explicit Init_SigFlags_pr_used(::ublox_ubx_msgs::msg::SigFlags & msg)
  : msg_(msg)
  {}
  Init_SigFlags_cr_used pr_used(::ublox_ubx_msgs::msg::SigFlags::_pr_used_type arg)
  {
    msg_.pr_used = std::move(arg);
    return Init_SigFlags_cr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigFlags msg_;
};

class Init_SigFlags_pr_smoothed
{
public:
  explicit Init_SigFlags_pr_smoothed(::ublox_ubx_msgs::msg::SigFlags & msg)
  : msg_(msg)
  {}
  Init_SigFlags_pr_used pr_smoothed(::ublox_ubx_msgs::msg::SigFlags::_pr_smoothed_type arg)
  {
    msg_.pr_smoothed = std::move(arg);
    return Init_SigFlags_pr_used(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigFlags msg_;
};

class Init_SigFlags_health
{
public:
  Init_SigFlags_health()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SigFlags_pr_smoothed health(::ublox_ubx_msgs::msg::SigFlags::_health_type arg)
  {
    msg_.health = std::move(arg);
    return Init_SigFlags_pr_smoothed(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigFlags msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::SigFlags>()
{
  return ublox_ubx_msgs::msg::builder::Init_SigFlags_health();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__BUILDER_HPP_
