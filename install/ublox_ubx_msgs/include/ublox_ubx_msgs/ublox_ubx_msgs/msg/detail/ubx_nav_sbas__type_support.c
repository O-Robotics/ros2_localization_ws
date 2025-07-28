// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ublox_ubx_msgs:msg/UBXNavSBAS.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ublox_ubx_msgs/msg/detail/ubx_nav_sbas__rosidl_typesupport_introspection_c.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_sbas__functions.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_sbas__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `service`
#include "ublox_ubx_msgs/msg/sbas_service.h"
// Member `service`
#include "ublox_ubx_msgs/msg/detail/sbas_service__rosidl_typesupport_introspection_c.h"
// Member `status_flags`
#include "ublox_ubx_msgs/msg/sbas_status_flags.h"
// Member `status_flags`
#include "ublox_ubx_msgs/msg/detail/sbas_status_flags__rosidl_typesupport_introspection_c.h"
// Member `sv_data`
#include "ublox_ubx_msgs/msg/sbas_sv_data.h"
// Member `sv_data`
#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ublox_ubx_msgs__msg__UBXNavSBAS__init(message_memory);
}

void ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_fini_function(void * message_memory)
{
  ublox_ubx_msgs__msg__UBXNavSBAS__fini(message_memory);
}

size_t ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__size_function__UBXNavSBAS__reserved_0(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_const_function__UBXNavSBAS__reserved_0(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_function__UBXNavSBAS__reserved_0(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__fetch_function__UBXNavSBAS__reserved_0(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_const_function__UBXNavSBAS__reserved_0(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__assign_function__UBXNavSBAS__reserved_0(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_function__UBXNavSBAS__reserved_0(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

size_t ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__size_function__UBXNavSBAS__sv_data(
  const void * untyped_member)
{
  const ublox_ubx_msgs__msg__SBASSvData__Sequence * member =
    (const ublox_ubx_msgs__msg__SBASSvData__Sequence *)(untyped_member);
  return member->size;
}

const void * ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_const_function__UBXNavSBAS__sv_data(
  const void * untyped_member, size_t index)
{
  const ublox_ubx_msgs__msg__SBASSvData__Sequence * member =
    (const ublox_ubx_msgs__msg__SBASSvData__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_function__UBXNavSBAS__sv_data(
  void * untyped_member, size_t index)
{
  ublox_ubx_msgs__msg__SBASSvData__Sequence * member =
    (ublox_ubx_msgs__msg__SBASSvData__Sequence *)(untyped_member);
  return &member->data[index];
}

void ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__fetch_function__UBXNavSBAS__sv_data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ublox_ubx_msgs__msg__SBASSvData * item =
    ((const ublox_ubx_msgs__msg__SBASSvData *)
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_const_function__UBXNavSBAS__sv_data(untyped_member, index));
  ublox_ubx_msgs__msg__SBASSvData * value =
    (ublox_ubx_msgs__msg__SBASSvData *)(untyped_value);
  *value = *item;
}

void ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__assign_function__UBXNavSBAS__sv_data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ublox_ubx_msgs__msg__SBASSvData * item =
    ((ublox_ubx_msgs__msg__SBASSvData *)
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_function__UBXNavSBAS__sv_data(untyped_member, index));
  const ublox_ubx_msgs__msg__SBASSvData * value =
    (const ublox_ubx_msgs__msg__SBASSvData *)(untyped_value);
  *item = *value;
}

bool ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__resize_function__UBXNavSBAS__sv_data(
  void * untyped_member, size_t size)
{
  ublox_ubx_msgs__msg__SBASSvData__Sequence * member =
    (ublox_ubx_msgs__msg__SBASSvData__Sequence *)(untyped_member);
  ublox_ubx_msgs__msg__SBASSvData__Sequence__fini(member);
  return ublox_ubx_msgs__msg__SBASSvData__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_member_array[10] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "itow",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, itow),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "geo",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, geo),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sys",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, sys),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "service",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, service),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cnt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, cnt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status_flags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, status_flags),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reserved_0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, reserved_0),  // bytes offset in struct
    NULL,  // default value
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__size_function__UBXNavSBAS__reserved_0,  // size() function pointer
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_const_function__UBXNavSBAS__reserved_0,  // get_const(index) function pointer
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_function__UBXNavSBAS__reserved_0,  // get(index) function pointer
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__fetch_function__UBXNavSBAS__reserved_0,  // fetch(index, &value) function pointer
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__assign_function__UBXNavSBAS__reserved_0,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sv_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXNavSBAS, sv_data),  // bytes offset in struct
    NULL,  // default value
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__size_function__UBXNavSBAS__sv_data,  // size() function pointer
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_const_function__UBXNavSBAS__sv_data,  // get_const(index) function pointer
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__get_function__UBXNavSBAS__sv_data,  // get(index) function pointer
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__fetch_function__UBXNavSBAS__sv_data,  // fetch(index, &value) function pointer
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__assign_function__UBXNavSBAS__sv_data,  // assign(index, value) function pointer
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__resize_function__UBXNavSBAS__sv_data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_members = {
  "ublox_ubx_msgs__msg",  // message namespace
  "UBXNavSBAS",  // message name
  10,  // number of fields
  sizeof(ublox_ubx_msgs__msg__UBXNavSBAS),
  ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_member_array,  // message members
  ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_init_function,  // function to initialize message memory (memory has to be allocated)
  ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_type_support_handle = {
  0,
  &ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ublox_ubx_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, UBXNavSBAS)() {
  ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, SBASService)();
  ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, SBASStatusFlags)();
  ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, SBASSvData)();
  if (!ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_type_support_handle.typesupport_identifier) {
    ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ublox_ubx_msgs__msg__UBXNavSBAS__rosidl_typesupport_introspection_c__UBXNavSBAS_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
