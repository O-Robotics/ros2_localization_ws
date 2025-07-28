// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/UtcStd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ublox_ubx_msgs/msg/detail/utc_std__struct.hpp"
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

void UtcStd_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ublox_ubx_msgs::msg::UtcStd(_init);
}

void UtcStd_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ublox_ubx_msgs::msg::UtcStd *>(message_memory);
  typed_message->~UtcStd();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember UtcStd_message_member_array[1] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs::msg::UtcStd, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers UtcStd_message_members = {
  "ublox_ubx_msgs::msg",  // message namespace
  "UtcStd",  // message name
  1,  // number of fields
  sizeof(ublox_ubx_msgs::msg::UtcStd),
  UtcStd_message_member_array,  // message members
  UtcStd_init_function,  // function to initialize message memory (memory has to be allocated)
  UtcStd_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t UtcStd_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &UtcStd_message_members,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::UtcStd>()
{
  return &::ublox_ubx_msgs::msg::rosidl_typesupport_introspection_cpp::UtcStd_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ublox_ubx_msgs, msg, UtcStd)() {
  return &::ublox_ubx_msgs::msg::rosidl_typesupport_introspection_cpp::UtcStd_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
