// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ublox_ubx_msgs:msg/OrbSVInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__rosidl_typesupport_introspection_c.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__functions.h"
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__struct.h"


// Include directives for member types
// Member `sv_flag`
#include "ublox_ubx_msgs/msg/orb_sv_flag.h"
// Member `sv_flag`
#include "ublox_ubx_msgs/msg/detail/orb_sv_flag__rosidl_typesupport_introspection_c.h"
// Member `eph`
#include "ublox_ubx_msgs/msg/orb_eph_info.h"
// Member `eph`
#include "ublox_ubx_msgs/msg/detail/orb_eph_info__rosidl_typesupport_introspection_c.h"
// Member `alm`
#include "ublox_ubx_msgs/msg/orb_alm_info.h"
// Member `alm`
#include "ublox_ubx_msgs/msg/detail/orb_alm_info__rosidl_typesupport_introspection_c.h"
// Member `other_orb`
#include "ublox_ubx_msgs/msg/other_orb_info.h"
// Member `other_orb`
#include "ublox_ubx_msgs/msg/detail/other_orb_info__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ublox_ubx_msgs__msg__OrbSVInfo__init(message_memory);
}

void ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_fini_function(void * message_memory)
{
  ublox_ubx_msgs__msg__OrbSVInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_member_array[6] = {
  {
    "gnss_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__OrbSVInfo, gnss_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sv_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__OrbSVInfo, sv_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sv_flag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__OrbSVInfo, sv_flag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "eph",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__OrbSVInfo, eph),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "alm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__OrbSVInfo, alm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "other_orb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_ubx_msgs__msg__OrbSVInfo, other_orb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_members = {
  "ublox_ubx_msgs__msg",  // message namespace
  "OrbSVInfo",  // message name
  6,  // number of fields
  sizeof(ublox_ubx_msgs__msg__OrbSVInfo),
  ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_member_array,  // message members
  ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_type_support_handle = {
  0,
  &ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ublox_ubx_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, OrbSVInfo)() {
  ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, OrbSVFlag)();
  ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, OrbEphInfo)();
  ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, OrbAlmInfo)();
  ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, OtherOrbInfo)();
  if (!ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_type_support_handle.typesupport_identifier) {
    ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ublox_ubx_msgs__msg__OrbSVInfo__rosidl_typesupport_introspection_c__OrbSVInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
