// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/UBXEsfMeas.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ublox_ubx_msgs/msg/detail/ubx_esf_meas__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ublox_ubx_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void UBXEsfMeas_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ublox_ubx_msgs::msg::UBXEsfMeas(_init);
}

void UBXEsfMeas_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ublox_ubx_msgs::msg::UBXEsfMeas *>(message_memory);
  typed_message->~UBXEsfMeas();
}

size_t size_function__UBXEsfMeas__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ublox_ubx_msgs::msg::ESFMeasDataItem> *>(untyped_member);
  return member->size();
}

const void * get_const_function__UBXEsfMeas__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ublox_ubx_msgs::msg::ESFMeasDataItem> *>(untyped_member);
  return &member[index];
}

void * get_function__UBXEsfMeas__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ublox_ubx_msgs::msg::ESFMeasDataItem> *>(untyped_member);
  return &member[index];
}

void fetch_function__UBXEsfMeas__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ublox_ubx_msgs::msg::ESFMeasDataItem *>(
    get_const_function__UBXEsfMeas__data(untyped_member, index));
  auto & value = *reinterpret_cast<ublox_ubx_msgs::msg::ESFMeasDataItem *>(untyped_value);
  value = item;
}

void assign_function__UBXEsfMeas__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ublox_ubx_msgs::msg::ESFMeasDataItem *>(
    get_function__UBXEsfMeas__data(untyped_member, index));
  const auto & value = *reinterpret_cast<const ublox_ubx_msgs::msg::ESFMeasDataItem *>(untyped_value);
  item = value;
}

void resize_function__UBXEsfMeas__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ublox_ubx_msgs::msg::ESFMeasDataItem> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember UBXEsfMeas_message_member_array[9] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UBXEsfMeas, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time_tag",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UBXEsfMeas, time_tag),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time_mark_sent",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UBXEsfMeas, time_mark_sent),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time_mark_edge",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UBXEsfMeas, time_mark_edge),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "calib_ttag_valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UBXEsfMeas, calib_ttag_valid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "num_meas",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UBXEsfMeas, num_meas),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UBXEsfMeas, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ublox_ubx_msgs::msg::ESFMeasDataItem>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UBXEsfMeas, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__UBXEsfMeas__data,  // size() function pointer
    get_const_function__UBXEsfMeas__data,  // get_const(index) function pointer
    get_function__UBXEsfMeas__data,  // get(index) function pointer
    fetch_function__UBXEsfMeas__data,  // fetch(index, &value) function pointer
    assign_function__UBXEsfMeas__data,  // assign(index, value) function pointer
    resize_function__UBXEsfMeas__data  // resize(index) function pointer
  },
  {
    "calib_ttag",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UBXEsfMeas, calib_ttag),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers UBXEsfMeas_message_members = {
  "ublox_ubx_msgs::msg",  // message namespace
  "UBXEsfMeas",  // message name
  9,  // number of fields
  sizeof(ublox_ubx_msgs::msg::UBXEsfMeas),
  UBXEsfMeas_message_member_array,  // message members
  UBXEsfMeas_init_function,  // function to initialize message memory (memory has to be allocated)
  UBXEsfMeas_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t UBXEsfMeas_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &UBXEsfMeas_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ublox_ubx_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ublox_ubx_msgs::msg::UBXEsfMeas>()
{
  return &::ublox_ubx_msgs::msg::rosidl_typesupport_introspection_cpp::UBXEsfMeas_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ublox_ubx_msgs, msg, UBXEsfMeas)() {
  return &::ublox_ubx_msgs::msg::rosidl_typesupport_introspection_cpp::UBXEsfMeas_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
