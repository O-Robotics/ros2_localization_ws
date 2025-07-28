// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/SatInfo.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sat_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/sat_info__struct.h"
#include "ublox_ubx_msgs/msg/detail/sat_info__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "ublox_ubx_msgs/msg/detail/sat_flags__functions.h"  // flags

// forward declare type support functions
size_t get_serialized_size_ublox_ubx_msgs__msg__SatFlags(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__SatFlags(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SatFlags)();


using _SatInfo__ros_msg_type = ublox_ubx_msgs__msg__SatInfo;

static bool _SatInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SatInfo__ros_msg_type * ros_message = static_cast<const _SatInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: gnss_id
  {
    cdr << ros_message->gnss_id;
  }

  // Field name: sv_id
  {
    cdr << ros_message->sv_id;
  }

  // Field name: cno
  {
    cdr << ros_message->cno;
  }

  // Field name: elev
  {
    cdr << ros_message->elev;
  }

  // Field name: azim
  {
    cdr << ros_message->azim;
  }

  // Field name: pr_res
  {
    cdr << ros_message->pr_res;
  }

  // Field name: flags
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SatFlags
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->flags, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _SatInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SatInfo__ros_msg_type * ros_message = static_cast<_SatInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: gnss_id
  {
    cdr >> ros_message->gnss_id;
  }

  // Field name: sv_id
  {
    cdr >> ros_message->sv_id;
  }

  // Field name: cno
  {
    cdr >> ros_message->cno;
  }

  // Field name: elev
  {
    cdr >> ros_message->elev;
  }

  // Field name: azim
  {
    cdr >> ros_message->azim;
  }

  // Field name: pr_res
  {
    cdr >> ros_message->pr_res;
  }

  // Field name: flags
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SatFlags
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->flags))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__SatInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SatInfo__ros_msg_type * ros_message = static_cast<const _SatInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name gnss_id
  {
    size_t item_size = sizeof(ros_message->gnss_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sv_id
  {
    size_t item_size = sizeof(ros_message->sv_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cno
  {
    size_t item_size = sizeof(ros_message->cno);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name elev
  {
    size_t item_size = sizeof(ros_message->elev);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name azim
  {
    size_t item_size = sizeof(ros_message->azim);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pr_res
  {
    size_t item_size = sizeof(ros_message->pr_res);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name flags

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__SatFlags(
    &(ros_message->flags), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _SatInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__SatInfo(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__SatInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: gnss_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sv_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cno
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: elev
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: azim
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: pr_res
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: flags
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__SatFlags(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ublox_ubx_msgs__msg__SatInfo;
    is_plain =
      (
      offsetof(DataType, flags) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SatInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__SatInfo(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SatInfo = {
  "ublox_ubx_msgs::msg",
  "SatInfo",
  _SatInfo__cdr_serialize,
  _SatInfo__cdr_deserialize,
  _SatInfo__get_serialized_size,
  _SatInfo__max_serialized_size
};

static rosidl_message_type_support_t _SatInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SatInfo,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SatInfo)() {
  return &_SatInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
