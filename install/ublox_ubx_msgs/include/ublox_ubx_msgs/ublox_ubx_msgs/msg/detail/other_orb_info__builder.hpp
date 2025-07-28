// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/OtherOrbInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__OTHER_ORB_INFO__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__OTHER_ORB_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/other_orb_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_OtherOrbInfo_orb_type
{
public:
  explicit Init_OtherOrbInfo_orb_type(::ublox_ubx_msgs::msg::OtherOrbInfo & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::OtherOrbInfo orb_type(::ublox_ubx_msgs::msg::OtherOrbInfo::_orb_type_type arg)
  {
    msg_.orb_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OtherOrbInfo msg_;
};

class Init_OtherOrbInfo_ano_aop_usability
{
public:
  Init_OtherOrbInfo_ano_aop_usability()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OtherOrbInfo_orb_type ano_aop_usability(::ublox_ubx_msgs::msg::OtherOrbInfo::_ano_aop_usability_type arg)
  {
    msg_.ano_aop_usability = std::move(arg);
    return Init_OtherOrbInfo_orb_type(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::OtherOrbInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::OtherOrbInfo>()
{
  return ublox_ubx_msgs::msg::builder::Init_OtherOrbInfo_ano_aop_usability();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__OTHER_ORB_INFO__BUILDER_HPP_
