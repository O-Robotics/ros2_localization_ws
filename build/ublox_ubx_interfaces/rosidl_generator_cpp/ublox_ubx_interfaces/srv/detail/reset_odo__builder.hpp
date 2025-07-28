// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_interfaces:srv/ResetODO.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_INTERFACES__SRV__DETAIL__RESET_ODO__BUILDER_HPP_
#define UBLOX_UBX_INTERFACES__SRV__DETAIL__RESET_ODO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_interfaces/srv/detail/reset_odo__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_interfaces::srv::ResetODO_Request>()
{
  return ::ublox_ubx_interfaces::srv::ResetODO_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
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
auto build<::ublox_ubx_interfaces::srv::ResetODO_Response>()
{
  return ::ublox_ubx_interfaces::srv::ResetODO_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ublox_ubx_interfaces

#endif  // UBLOX_UBX_INTERFACES__SRV__DETAIL__RESET_ODO__BUILDER_HPP_
