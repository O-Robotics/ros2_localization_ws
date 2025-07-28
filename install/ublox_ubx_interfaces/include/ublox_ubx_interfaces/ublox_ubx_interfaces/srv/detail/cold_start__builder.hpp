// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_interfaces:srv/ColdStart.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_INTERFACES__SRV__DETAIL__COLD_START__BUILDER_HPP_
#define UBLOX_UBX_INTERFACES__SRV__DETAIL__COLD_START__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_interfaces/srv/detail/cold_start__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_interfaces
{

namespace srv
{

namespace builder
{

class Init_ColdStart_Request_reset_type
{
public:
  Init_ColdStart_Request_reset_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ublox_ubx_interfaces::srv::ColdStart_Request reset_type(::ublox_ubx_interfaces::srv::ColdStart_Request::_reset_type_type arg)
  {
    msg_.reset_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_interfaces::srv::ColdStart_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_interfaces::srv::ColdStart_Request>()
{
  return ublox_ubx_interfaces::srv::builder::Init_ColdStart_Request_reset_type();
}

}  // namespace ublox_ubx_interfaces


namespace ublox_ubx_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_interfaces::srv::ColdStart_Response>()
{
  return ::ublox_ubx_interfaces::srv::ColdStart_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ublox_ubx_interfaces

#endif  // UBLOX_UBX_INTERFACES__SRV__DETAIL__COLD_START__BUILDER_HPP_
