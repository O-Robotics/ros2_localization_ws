// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXSecUniqid.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_UNIQID__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_UNIQID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_sec_uniqid__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXSecUniqid_unique_id
{
public:
  explicit Init_UBXSecUniqid_unique_id(::ublox_ubx_msgs::msg::UBXSecUniqid & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXSecUniqid unique_id(::ublox_ubx_msgs::msg::UBXSecUniqid::_unique_id_type arg)
  {
    msg_.unique_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecUniqid msg_;
};

class Init_UBXSecUniqid_version
{
public:
  explicit Init_UBXSecUniqid_version(::ublox_ubx_msgs::msg::UBXSecUniqid & msg)
  : msg_(msg)
  {}
  Init_UBXSecUniqid_unique_id version(::ublox_ubx_msgs::msg::UBXSecUniqid::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXSecUniqid_unique_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecUniqid msg_;
};

class Init_UBXSecUniqid_header
{
public:
  Init_UBXSecUniqid_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXSecUniqid_version header(::ublox_ubx_msgs::msg::UBXSecUniqid::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXSecUniqid_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXSecUniqid msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXSecUniqid>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXSecUniqid_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_UNIQID__BUILDER_HPP_
