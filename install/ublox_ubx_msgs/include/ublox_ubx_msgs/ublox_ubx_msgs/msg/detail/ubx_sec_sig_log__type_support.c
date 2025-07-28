// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ublox_ubx_msgs:msg/UBXSecSigLog.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ublox_ubx_msgs/msg/detail/ubx_sec_sig_log__rosidl_typesupport_introspection_c.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ublox_ubx_msgs/msg/detail/ubx_sec_sig_log__functions.h"
#include "ublox_ubx_msgs/msg/detail/ubx_sec_sig_log__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `events`
#include "ublox_ubx_msgs/msg/sig_log_event.h"
// Member `events`
#include "ublox_ubx_msgs/msg/detail/sig_log_event__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ublox_ubx_msgs__msg__UBXSecSigLog__init(message_memory);
}

void ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_fini_function(void * message_memory)
{
  ublox_ubx_msgs__msg__UBXSecSigLog__fini(message_memory);
}

size_t ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__size_function__UBXSecSigLog__events(
  const void * untyped_member)
{
  const ublox_ubx_msgs__msg__SigLogEvent__Sequence * member =
    (const ublox_ubx_msgs__msg__SigLogEvent__Sequence *)(untyped_member);
  return member->size;
}

const void * ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__get_const_function__UBXSecSigLog__events(
  const void * untyped_member, size_t index)
{
  const ublox_ubx_msgs__msg__SigLogEvent__Sequence * member =
    (const ublox_ubx_msgs__msg__SigLogEvent__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__get_function__UBXSecSigLog__events(
  void * untyped_member, size_t index)
{
  ublox_ubx_msgs__msg__SigLogEvent__Sequence * member =
    (ublox_ubx_msgs__msg__SigLogEvent__Sequence *)(untyped_member);
  return &member->data[index];
}

void ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__fetch_function__UBXSecSigLog__events(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ublox_ubx_msgs__msg__SigLogEvent * item =
    ((const ublox_ubx_msgs__msg__SigLogEvent *)
    ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__get_const_function__UBXSecSigLog__events(untyped_member, index));
  ublox_ubx_msgs__msg__SigLogEvent * value =
    (ublox_ubx_msgs__msg__SigLogEvent *)(untyped_value);
  *value = *item;
}

void ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__assign_function__UBXSecSigLog__events(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ublox_ubx_msgs__msg__SigLogEvent * item =
    ((ublox_ubx_msgs__msg__SigLogEvent *)
    ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__get_function__UBXSecSigLog__events(untyped_member, index));
  const ublox_ubx_msgs__msg__SigLogEvent * value =
    (const ublox_ubx_msgs__msg__SigLogEvent *)(untyped_value);
  *item = *value;
}

bool ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__resize_function__UBXSecSigLog__events(
  void * untyped_member, size_t size)
{
  ublox_ubx_msgs__msg__SigLogEvent__Sequence * member =
    (ublox_ubx_msgs__msg__SigLogEvent__Sequence *)(untyped_member);
  ublox_ubx_msgs__msg__SigLogEvent__Sequence__fini(member);
  return ublox_ubx_msgs__msg__SigLogEvent__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXSecSigLog, header),  // bytes offset in struct
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
    offsetof(ublox_ubx_msgs__msg__UBXSecSigLog, version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_events",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXSecSigLog, num_events),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "events",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXSecSigLog, events),  // bytes offset in struct
    NULL,  // default value
    ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__size_function__UBXSecSigLog__events,  // size() function pointer
    ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__get_const_function__UBXSecSigLog__events,  // get_const(index) function pointer
    ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__get_function__UBXSecSigLog__events,  // get(index) function pointer
    ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__fetch_function__UBXSecSigLog__events,  // fetch(index, &value) function pointer
    ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__assign_function__UBXSecSigLog__events,  // assign(index, value) function pointer
    ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__resize_function__UBXSecSigLog__events  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_members = {
  "ublox_ubx_msgs__msg",  // message namespace
  "UBXSecSigLog",  // message name
  4,  // number of fields
  sizeof(ublox_ubx_msgs__msg__UBXSecSigLog),
  ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_member_array,  // message members
  ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_init_function,  // function to initialize message memory (memory has to be allocated)
  ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_type_support_handle = {
  0,
  &ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ublox_ubx_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, UBXSecSigLog)() {
  ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, SigLogEvent)();
  if (!ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_type_support_handle.typesupport_identifier) {
    ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ublox_ubx_msgs__msg__UBXSecSigLog__rosidl_typesupport_introspection_c__UBXSecSigLog_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
