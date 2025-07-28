// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXSecSig.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_sec_sig__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXSecSig_spoofing_state
{
public:
  explicit Init_UBXSecSig_spoofing_state(::ublox_ubx_msgs::msg::UBXSecSig & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXSecSig spoofing_state(::ublox_ubx_msgs::msg::UBXSecSig::_spoofing_state_type arg)
  {
    msg_.spoofing_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSig msg_;
};

class Init_UBXSecSig_spf_det_enabled
{
public:
  explicit Init_UBXSecSig_spf_det_enabled(::ublox_ubx_msgs::msg::UBXSecSig & msg)
  : msg_(msg)
  {}
  Init_UBXSecSig_spoofing_state spf_det_enabled(::ublox_ubx_msgs::msg::UBXSecSig::_spf_det_enabled_type arg)
  {
    msg_.spf_det_enabled = std::move(arg);
    return Init_UBXSecSig_spoofing_state(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSig msg_;
};

class Init_UBXSecSig_jamming_state
{
public:
  explicit Init_UBXSecSig_jamming_state(::ublox_ubx_msgs::msg::UBXSecSig & msg)
  : msg_(msg)
  {}
  Init_UBXSecSig_spf_det_enabled jamming_state(::ublox_ubx_msgs::msg::UBXSecSig::_jamming_state_type arg)
  {
    msg_.jamming_state = std::move(arg);
    return Init_UBXSecSig_spf_det_enabled(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSig msg_;
};

class Init_UBXSecSig_jam_det_enabled
{
public:
  explicit Init_UBXSecSig_jam_det_enabled(::ublox_ubx_msgs::msg::UBXSecSig & msg)
  : msg_(msg)
  {}
  Init_UBXSecSig_jamming_state jam_det_enabled(::ublox_ubx_msgs::msg::UBXSecSig::_jam_det_enabled_type arg)
  {
    msg_.jam_det_enabled = std::move(arg);
    return Init_UBXSecSig_jamming_state(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSig msg_;
};

class Init_UBXSecSig_version
{
public:
  explicit Init_UBXSecSig_version(::ublox_ubx_msgs::msg::UBXSecSig & msg)
  : msg_(msg)
  {}
  Init_UBXSecSig_jam_det_enabled version(::ublox_ubx_msgs::msg::UBXSecSig::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXSecSig_jam_det_enabled(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSig msg_;
};

class Init_UBXSecSig_header
{
public:
  Init_UBXSecSig_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXSecSig_version header(::ublox_ubx_msgs::msg::UBXSecSig::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXSecSig_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecSig msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXSecSig>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXSecSig_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__BUILDER_HPP_
