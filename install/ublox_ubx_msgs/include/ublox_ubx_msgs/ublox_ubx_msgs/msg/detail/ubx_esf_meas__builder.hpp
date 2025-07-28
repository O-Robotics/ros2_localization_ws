// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXEsfMeas.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_MEAS__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_MEAS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_esf_meas__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXEsfMeas_calib_ttag
{
public:
  explicit Init_UBXEsfMeas_calib_ttag(::ublox_ubx_msgs::msg::UBXEsfMeas & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXEsfMeas calib_ttag(::ublox_ubx_msgs::msg::UBXEsfMeas::_calib_ttag_type arg)
  {
    msg_.calib_ttag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfMeas msg_;
};

class Init_UBXEsfMeas_data
{
public:
  explicit Init_UBXEsfMeas_data(::ublox_ubx_msgs::msg::UBXEsfMeas & msg)
  : msg_(msg)
  {}
  Init_UBXEsfMeas_calib_ttag data(::ublox_ubx_msgs::msg::UBXEsfMeas::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_UBXEsfMeas_calib_ttag(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfMeas msg_;
};

class Init_UBXEsfMeas_id
{
public:
  explicit Init_UBXEsfMeas_id(::ublox_ubx_msgs::msg::UBXEsfMeas & msg)
  : msg_(msg)
  {}
  Init_UBXEsfMeas_data id(::ublox_ubx_msgs::msg::UBXEsfMeas::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_UBXEsfMeas_data(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfMeas msg_;
};

class Init_UBXEsfMeas_num_meas
{
public:
  explicit Init_UBXEsfMeas_num_meas(::ublox_ubx_msgs::msg::UBXEsfMeas & msg)
  : msg_(msg)
  {}
  Init_UBXEsfMeas_id num_meas(::ublox_ubx_msgs::msg::UBXEsfMeas::_num_meas_type arg)
  {
    msg_.num_meas = std::move(arg);
    return Init_UBXEsfMeas_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfMeas msg_;
};

class Init_UBXEsfMeas_calib_ttag_valid
{
public:
  explicit Init_UBXEsfMeas_calib_ttag_valid(::ublox_ubx_msgs::msg::UBXEsfMeas & msg)
  : msg_(msg)
  {}
  Init_UBXEsfMeas_num_meas calib_ttag_valid(::ublox_ubx_msgs::msg::UBXEsfMeas::_calib_ttag_valid_type arg)
  {
    msg_.calib_ttag_valid = std::move(arg);
    return Init_UBXEsfMeas_num_meas(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfMeas msg_;
};

class Init_UBXEsfMeas_time_mark_edge
{
public:
  explicit Init_UBXEsfMeas_time_mark_edge(::ublox_ubx_msgs::msg::UBXEsfMeas & msg)
  : msg_(msg)
  {}
  Init_UBXEsfMeas_calib_ttag_valid time_mark_edge(::ublox_ubx_msgs::msg::UBXEsfMeas::_time_mark_edge_type arg)
  {
    msg_.time_mark_edge = std::move(arg);
    return Init_UBXEsfMeas_calib_ttag_valid(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfMeas msg_;
};

class Init_UBXEsfMeas_time_mark_sent
{
public:
  explicit Init_UBXEsfMeas_time_mark_sent(::ublox_ubx_msgs::msg::UBXEsfMeas & msg)
  : msg_(msg)
  {}
  Init_UBXEsfMeas_time_mark_edge time_mark_sent(::ublox_ubx_msgs::msg::UBXEsfMeas::_time_mark_sent_type arg)
  {
    msg_.time_mark_sent = std::move(arg);
    return Init_UBXEsfMeas_time_mark_edge(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfMeas msg_;
};

class Init_UBXEsfMeas_time_tag
{
public:
  explicit Init_UBXEsfMeas_time_tag(::ublox_ubx_msgs::msg::UBXEsfMeas & msg)
  : msg_(msg)
  {}
  Init_UBXEsfMeas_time_mark_sent time_tag(::ublox_ubx_msgs::msg::UBXEsfMeas::_time_tag_type arg)
  {
    msg_.time_tag = std::move(arg);
    return Init_UBXEsfMeas_time_mark_sent(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfMeas msg_;
};

class Init_UBXEsfMeas_header
{
public:
  Init_UBXEsfMeas_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXEsfMeas_time_tag header(::ublox_ubx_msgs::msg::UBXEsfMeas::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXEsfMeas_time_tag(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXEsfMeas msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXEsfMeas>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXEsfMeas_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_MEAS__BUILDER_HPP_
