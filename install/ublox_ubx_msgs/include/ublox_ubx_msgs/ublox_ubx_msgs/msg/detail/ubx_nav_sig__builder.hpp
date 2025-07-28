// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavSig.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SIG__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_sig__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavSig_sig_data
{
public:
  explicit Init_UBXNavSig_sig_data(::ublox_ubx_msgs::msg::UBXNavSig & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavSig sig_data(::ublox_ubx_msgs::msg::UBXNavSig::_sig_data_type arg)
  {
    msg_.sig_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSig msg_;
};

class Init_UBXNavSig_reserved_0
{
public:
  explicit Init_UBXNavSig_reserved_0(::ublox_ubx_msgs::msg::UBXNavSig & msg)
  : msg_(msg)
  {}
  Init_UBXNavSig_sig_data reserved_0(::ublox_ubx_msgs::msg::UBXNavSig::_reserved_0_type arg)
  {
    msg_.reserved_0 = std::move(arg);
    return Init_UBXNavSig_sig_data(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSig msg_;
};

class Init_UBXNavSig_num_sigs
{
public:
  explicit Init_UBXNavSig_num_sigs(::ublox_ubx_msgs::msg::UBXNavSig & msg)
  : msg_(msg)
  {}
  Init_UBXNavSig_reserved_0 num_sigs(::ublox_ubx_msgs::msg::UBXNavSig::_num_sigs_type arg)
  {
    msg_.num_sigs = std::move(arg);
    return Init_UBXNavSig_reserved_0(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSig msg_;
};

class Init_UBXNavSig_version
{
public:
  explicit Init_UBXNavSig_version(::ublox_ubx_msgs::msg::UBXNavSig & msg)
  : msg_(msg)
  {}
  Init_UBXNavSig_num_sigs version(::ublox_ubx_msgs::msg::UBXNavSig::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXNavSig_num_sigs(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSig msg_;
};

class Init_UBXNavSig_itow
{
public:
  explicit Init_UBXNavSig_itow(::ublox_ubx_msgs::msg::UBXNavSig & msg)
  : msg_(msg)
  {}
  Init_UBXNavSig_version itow(::ublox_ubx_msgs::msg::UBXNavSig::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavSig_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSig msg_;
};

class Init_UBXNavSig_header
{
public:
  Init_UBXNavSig_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavSig_itow header(::ublox_ubx_msgs::msg::UBXNavSig::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavSig_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSig msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavSig>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavSig_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SIG__BUILDER_HPP_
