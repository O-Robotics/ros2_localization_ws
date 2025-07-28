// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/SigLogEvent.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/sig_log_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_SigLogEvent_event_type
{
public:
  explicit Init_SigLogEvent_event_type(::ublox_ubx_msgs::msg::SigLogEvent & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::SigLogEvent event_type(::ublox_ubx_msgs::msg::SigLogEvent::_event_type_type arg)
  {
    msg_.event_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigLogEvent msg_;
};

class Init_SigLogEvent_detection_type
{
public:
  explicit Init_SigLogEvent_detection_type(::ublox_ubx_msgs::msg::SigLogEvent & msg)
  : msg_(msg)
  {}
  Init_SigLogEvent_event_type detection_type(::ublox_ubx_msgs::msg::SigLogEvent::_detection_type_type arg)
  {
    msg_.detection_type = std::move(arg);
    return Init_SigLogEvent_event_type(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigLogEvent msg_;
};

class Init_SigLogEvent_time_elapsed
{
public:
  Init_SigLogEvent_time_elapsed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SigLogEvent_detection_type time_elapsed(::ublox_ubx_msgs::msg::SigLogEvent::_time_elapsed_type arg)
  {
    msg_.time_elapsed = std::move(arg);
    return Init_SigLogEvent_detection_type(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::SigLogEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::SigLogEvent>()
{
  return ublox_ubx_msgs::msg::builder::Init_SigLogEvent_time_elapsed();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__BUILDER_HPP_
