// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/CarrSoln.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__CARR_SOLN__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__CARR_SOLN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/carr_soln__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_CarrSoln_status
{
public:
  Init_CarrSoln_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ublox_ubx_msgs::msg::CarrSoln status(::ublox_ubx_msgs::msg::CarrSoln::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::CarrSoln msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::CarrSoln>()
{
  return ublox_ubx_msgs::msg::builder::Init_CarrSoln_status();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__CARR_SOLN__BUILDER_HPP_
