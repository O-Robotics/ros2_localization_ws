// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ublox_ubx_msgs:msg/UBXEsfMeas.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ublox_ubx_msgs/msg/detail/ubx_esf_meas__rosidl_typesupport_introspection_c.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ublox_ubx_msgs/msg/detail/ubx_esf_meas__functions.h"
#include "ublox_ubx_msgs/msg/detail/ubx_esf_meas__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "ublox_ubx_msgs/msg/esf_meas_data_item.h"
// Member `data`
#include "ublox_ubx_msgs/msg/detail/esf_meas_data_item__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ublox_ubx_msgs__msg__UBXEsfMeas__init(message_memory);
}

void ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_fini_function(void * message_memory)
{
  ublox_ubx_msgs__msg__UBXEsfMeas__fini(message_memory);
}

size_t ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__size_function__UBXEsfMeas__data(
  const void * untyped_member)
{
  const ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence * member =
    (const ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence *)(untyped_member);
  return member->size;
}

const void * ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__get_const_function__UBXEsfMeas__data(
  const void * untyped_member, size_t index)
{
  const ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence * member =
    (const ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__get_function__UBXEsfMeas__data(
  void * untyped_member, size_t index)
{
  ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence * member =
    (ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence *)(untyped_member);
  return &member->data[index];
}

void ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__fetch_function__UBXEsfMeas__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ublox_ubx_msgs__msg__ESFMeasDataItem * item =
    ((const ublox_ubx_msgs__msg__ESFMeasDataItem *)
    ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__get_const_function__UBXEsfMeas__data(untyped_member, index));
  ublox_ubx_msgs__msg__ESFMeasDataItem * value =
    (ublox_ubx_msgs__msg__ESFMeasDataItem *)(untyped_value);
  *value = *item;
}

void ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__assign_function__UBXEsfMeas__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ublox_ubx_msgs__msg__ESFMeasDataItem * item =
    ((ublox_ubx_msgs__msg__ESFMeasDataItem *)
    ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__get_function__UBXEsfMeas__data(untyped_member, index));
  const ublox_ubx_msgs__msg__ESFMeasDataItem * value =
    (const ublox_ubx_msgs__msg__ESFMeasDataItem *)(untyped_value);
  *item = *value;
}

bool ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__resize_function__UBXEsfMeas__data(
  void * untyped_member, size_t size)
{
  ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence * member =
    (ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence *)(untyped_member);
  ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence__fini(member);
  return ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXEsfMeas, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_tag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXEsfMeas, time_tag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_mark_sent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXEsfMeas, time_mark_sent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_mark_edge",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXEsfMeas, time_mark_edge),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "calib_ttag_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXEsfMeas, calib_ttag_valid),  // bytes offset in struct
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
    offsetof(ublox_ubx_msgs__msg__UBXEsfMeas, num_meas),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXEsfMeas, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXEsfMeas, data),  // bytes offset in struct
    NULL,  // default value
    ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__size_function__UBXEsfMeas__data,  // size() function pointer
    ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__get_const_function__UBXEsfMeas__data,  // get_const(index) function pointer
    ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__get_function__UBXEsfMeas__data,  // get(index) function pointer
    ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__fetch_function__UBXEsfMeas__data,  // fetch(index, &value) function pointer
    ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__assign_function__UBXEsfMeas__data,  // assign(index, value) function pointer
    ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__resize_function__UBXEsfMeas__data  // resize(index) function pointer
  },
  {
    "calib_ttag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__UBXEsfMeas, calib_ttag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_members = {
  "ublox_ubx_msgs__msg",  // message namespace
  "UBXEsfMeas",  // message name
  9,  // number of fields
  sizeof(ublox_ubx_msgs__msg__UBXEsfMeas),
  ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_member_array,  // message members
  ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_init_function,  // function to initialize message memory (memory has to be allocated)
  ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_type_support_handle = {
  0,
  &ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ublox_ubx_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, UBXEsfMeas)() {
  ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, ESFMeasDataItem)();
  if (!ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_type_support_handle.typesupport_identifier) {
    ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ublox_ubx_msgs__msg__UBXEsfMeas__rosidl_typesupport_introspection_c__UBXEsfMeas_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
