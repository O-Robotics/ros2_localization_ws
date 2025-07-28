// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/SigFlags.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sig_flags__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/sig_flags__struct.h"
#include "ublox_ubx_msgs/msg/detail/sig_flags__functions.h"
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


// forward declare type support functions


using _SigFlags__ros_msg_type = ublox_ubx_msgs__msg__SigFlags;

static bool _SigFlags__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SigFlags__ros_msg_type * ros_message = static_cast<const _SigFlags__ros_msg_type *>(untyped_ros_message);
  // Field name: health
  {
    cdr << ros_message->health;
  }

  // Field name: pr_smoothed
  {
    cdr << (ros_message->pr_smoothed ? true : false);
  }

  // Field name: pr_used
  {
    cdr << (ros_message->pr_used ? true : false);
  }

  // Field name: cr_used
  {
    cdr << (ros_message->cr_used ? true : false);
  }

  // Field name: do_used
  {
    cdr << (ros_message->do_used ? true : false);
  }

  // Field name: pr_corr_used
  {
    cdr << (ros_message->pr_corr_used ? true : false);
  }

  // Field name: cr_corr_used
  {
    cdr << (ros_message->cr_corr_used ? true : false);
  }

  // Field name: do_corr_used
  {
    cdr << (ros_message->do_corr_used ? true : false);
  }

  return true;
}

static bool _SigFlags__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SigFlags__ros_msg_type * ros_message = static_cast<_SigFlags__ros_msg_type *>(untyped_ros_message);
  // Field name: health
  {
    cdr >> ros_message->health;
  }

  // Field name: pr_smoothed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->pr_smoothed = tmp ? true : false;
  }

  // Field name: pr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->pr_used = tmp ? true : false;
  }

  // Field name: cr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->cr_used = tmp ? true : false;
  }

  // Field name: do_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->do_used = tmp ? true : false;
  }

  // Field name: pr_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->pr_corr_used = tmp ? true : false;
  }

  // Field name: cr_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->cr_corr_used = tmp ? true : false;
  }

  // Field name: do_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->do_corr_used = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__SigFlags(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SigFlags__ros_msg_type * ros_message = static_cast<const _SigFlags__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name health
  {
    size_t item_size = sizeof(ros_message->health);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pr_smoothed
  {
    size_t item_size = sizeof(ros_message->pr_smoothed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pr_used
  {
    size_t item_size = sizeof(ros_message->pr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cr_used
  {
    size_t item_size = sizeof(ros_message->cr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name do_used
  {
    size_t item_size = sizeof(ros_message->do_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pr_corr_used
  {
    size_t item_size = sizeof(ros_message->pr_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cr_corr_used
  {
    size_t item_size = sizeof(ros_message->cr_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name do_corr_used
  {
    size_t item_size = sizeof(ros_message->do_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SigFlags__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__SigFlags(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__SigFlags(
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

  // member: health
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pr_smoothed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: do_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pr_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cr_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: do_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ublox_ubx_msgs__msg__SigFlags;
    is_plain =
      (
      offsetof(DataType, do_corr_used) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SigFlags__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__SigFlags(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SigFlags = {
  "ublox_ubx_msgs::msg",
  "SigFlags",
  _SigFlags__cdr_serialize,
  _SigFlags__cdr_deserialize,
  _SigFlags__get_serialized_size,
  _SigFlags__max_serialized_size
};

static rosidl_message_type_support_t _SigFlags__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SigFlags,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SigFlags)() {
  return &_SigFlags__type_support;
}

#if defined(__cplusplus)
}
#endif
