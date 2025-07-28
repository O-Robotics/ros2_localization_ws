// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ublox_ubx_msgs:msg/UBXRxmRawx.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__rosidl_typesupport_introspection_c.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__functions.h"
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `rec_stat`
#include "ublox_ubx_msgs/msg/rec_stat.h"
// Member `rec_stat`
#include "ublox_ubx_msgs/msg/detail/rec_stat__rosidl_typesupport_introspection_c.h"
// Member `rawx_data`
#include "ublox_ubx_msgs/msg/rawx_data.h"
// Member `rawx_data`
#include "ublox_ubx_msgs/msg/detail/rawx_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ublox_ubx_msgs__msg__UBXRxmRawx__init(message_memory);
}

void ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_fini_function(void * message_memory)
{
  ublox_ubx_msgs__msg__UBXRxmRawx__fini(message_memory);
}

size_t ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__size_function__UBXRxmRawx__rawx_data(
  const void * untyped_member)
{
  const ublox_ubx_msgs__msg__RawxData__Sequence * member =
    (const ublox_ubx_msgs__msg__RawxData__Sequence *)(untyped_member);
  return member->size;
}

const void * ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__get_const_function__UBXRxmRawx__rawx_data(
  const void * untyped_member, size_t index)
{
  const ublox_ubx_msgs__msg__RawxData__Sequence * member =
    (const ublox_ubx_msgs__msg__RawxData__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__get_function__UBXRxmRawx__rawx_data(
  void * untyped_member, size_t index)
{
  ublox_ubx_msgs__msg__RawxData__Sequence * member =
    (ublox_ubx_msgs__msg__RawxData__Sequence *)(untyped_member);
  return &member->data[index];
}

void ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__fetch_function__UBXRxmRawx__rawx_data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ublox_ubx_msgs__msg__RawxData * item =
    ((const ublox_ubx_msgs__msg__RawxData *)
    ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__get_const_function__UBXRxmRawx__rawx_data(untyped_member, index));
  ublox_ubx_msgs__msg__RawxData * value =
    (ublox_ubx_msgs__msg__RawxData *)(untyped_value);
  *value = *item;
}

void ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__assign_function__UBXRxmRawx__rawx_data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ublox_ubx_msgs__msg__RawxData * item =
    ((ublox_ubx_msgs__msg__RawxData *)
    ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__get_function__UBXRxmRawx__rawx_data(untyped_member, index));
  const ublox_ubx_msgs__msg__RawxData * value =
    (const ublox_ubx_msgs__msg__RawxData *)(untyped_value);
  *item = *value;
}

bool ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__resize_function__UBXRxmRawx__rawx_data(
  void * untyped_member, size_t size)
{
  ublox_ubx_msgs__msg__RawxData__Sequence * member =
    (ublox_ubx_msgs__msg__RawxData__Sequence *)(untyped_member);
  ublox_ubx_msgs__msg__RawxData__Sequence__fini(member);
  return ublox_ubx_msgs__msg__RawxData__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXRxmRawx, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rcv_tow",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXRxmRawx, rcv_tow),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "week",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXRxmRawx, week),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "leap_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXRxmRawx, leap_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_meas",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXRxmRawx, num_meas),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rec_stat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXRxmRawx, rec_stat),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXRxmRawx, version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rawx_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXRxmRawx, rawx_data),  // bytes offset in struct
    NULL,  // default value
    ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__size_function__UBXRxmRawx__rawx_data,  // size() function pointer
    ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__get_const_function__UBXRxmRawx__rawx_data,  // get_const(index) function pointer
    ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__get_function__UBXRxmRawx__rawx_data,  // get(index) function pointer
    ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__fetch_function__UBXRxmRawx__rawx_data,  // fetch(index, &value) function pointer
    ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__assign_function__UBXRxmRawx__rawx_data,  // assign(index, value) function pointer
    ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__resize_function__UBXRxmRawx__rawx_data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_members = {
  "ublox_ubx_msgs__msg",  // message namespace
  "UBXRxmRawx",  // message name
  8,  // number of fields
  sizeof(ublox_ubx_msgs__msg__UBXRxmRawx),
  ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_member_array,  // message members
  ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_init_function,  // function to initialize message memory (memory has to be allocated)
  ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_type_support_handle = {
  0,
  &ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ublox_ubx_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, UBXRxmRawx)() {
  ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, RecStat)();
  ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, RawxData)();
  if (!ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_type_support_handle.typesupport_identifier) {
    ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ublox_ubx_msgs__msg__UBXRxmRawx__rosidl_typesupport_introspection_c__UBXRxmRawx_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
