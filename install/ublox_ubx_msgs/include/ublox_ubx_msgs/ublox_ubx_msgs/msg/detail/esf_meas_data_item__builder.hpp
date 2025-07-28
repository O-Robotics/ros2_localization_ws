// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/ESFMeasDataItem.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/esf_meas_data_item__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_ESFMeasDataItem_data_type
{
public:
  explicit Init_ESFMeasDataItem_data_type(::ublox_ubx_msgs::msg::ESFMeasDataItem & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::ESFMeasDataItem data_type(::ublox_ubx_msgs::msg::ESFMeasDataItem::_data_type_type arg)
  {
    msg_.data_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFMeasDataItem msg_;
};

class Init_ESFMeasDataItem_data_field
{
public:
  Init_ESFMeasDataItem_data_field()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESFMeasDataItem_data_type data_field(::ublox_ubx_msgs::msg::ESFMeasDataItem::_data_field_type arg)
  {
    msg_.data_field = std::move(arg);
    return Init_ESFMeasDataItem_data_type(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::ESFMeasDataItem msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::ESFMeasDataItem>()
{
  return ublox_ubx_msgs::msg::builder::Init_ESFMeasDataItem_data_field();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__BUILDER_HPP_
