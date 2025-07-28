// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/SigData.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sig_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/sig_data__struct.h"
#include "ublox_ubx_msgs/msg/detail/sig_data__functions.h"
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

#include "ublox_ubx_msgs/msg/detail/sig_flags__functions.h"  // sig_flags

// forward declare type support functions
size_t get_serialized_size_ublox_ubx_msgs__msg__SigFlags(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__SigFlags(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SigFlags)();


using _SigData__ros_msg_type = ublox_ubx_msgs__msg__SigData;

static bool _SigData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SigData__ros_msg_type * ros_message = static_cast<const _SigData__ros_msg_type *>(untyped_ros_message);
  // Field name: gnss_id
  {
    cdr << ros_message->gnss_id;
  }

  // Field name: sv_id
  {
    cdr << ros_message->sv_id;
  }

  // Field name: sig_id
  {
    cdr << ros_message->sig_id;
  }

  // Field name: freq_id
  {
    cdr << ros_message->freq_id;
  }

  // Field name: pr_res
  {
    cdr << ros_message->pr_res;
  }

  // Field name: cno
  {
    cdr << ros_message->cno;
  }

  // Field name: quality_ind
  {
    cdr << ros_message->quality_ind;
  }

  // Field name: corr_source
  {
    cdr << ros_message->corr_source;
  }

  // Field name: iono_model
  {
    cdr << ros_message->iono_model;
  }

  // Field name: sig_flags
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SigFlags
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->sig_flags, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _SigData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SigData__ros_msg_type * ros_message = static_cast<_SigData__ros_msg_type *>(untyped_ros_message);
  // Field name: gnss_id
  {
    cdr >> ros_message->gnss_id;
  }

  // Field name: sv_id
  {
    cdr >> ros_message->sv_id;
  }

  // Field name: sig_id
  {
    cdr >> ros_message->sig_id;
  }

  // Field name: freq_id
  {
    cdr >> ros_message->freq_id;
  }

  // Field name: pr_res
  {
    cdr >> ros_message->pr_res;
  }

  // Field name: cno
  {
    cdr >> ros_message->cno;
  }

  // Field name: quality_ind
  {
    cdr >> ros_message->quality_ind;
  }

  // Field name: corr_source
  {
    cdr >> ros_message->corr_source;
  }

  // Field name: iono_model
  {
    cdr >> ros_message->iono_model;
  }

  // Field name: sig_flags
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SigFlags
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->sig_flags))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__SigData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SigData__ros_msg_type * ros_message = static_cast<const _SigData__ros_msg_type *>(untyped_ros_message);
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
  // field.name sig_id
  {
    size_t item_size = sizeof(ros_message->sig_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name freq_id
  {
    size_t item_size = sizeof(ros_message->freq_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pr_res
  {
    size_t item_size = sizeof(ros_message->pr_res);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cno
  {
    size_t item_size = sizeof(ros_message->cno);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name quality_ind
  {
    size_t item_size = sizeof(ros_message->quality_ind);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name corr_source
  {
    size_t item_size = sizeof(ros_message->corr_source);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name iono_model
  {
    size_t item_size = sizeof(ros_message->iono_model);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sig_flags

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__SigFlags(
    &(ros_message->sig_flags), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _SigData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__SigData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__SigData(
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
  // member: sig_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: freq_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pr_res
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: cno
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: quality_ind
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: corr_source
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: iono_model
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sig_flags
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__SigFlags(
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
    using DataType = ublox_ubx_msgs__msg__SigData;
    is_plain =
      (
      offsetof(DataType, sig_flags) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SigData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__SigData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SigData = {
  "ublox_ubx_msgs::msg",
  "SigData",
  _SigData__cdr_serialize,
  _SigData__cdr_deserialize,
  _SigData__get_serialized_size,
  _SigData__max_serialized_size
};

static rosidl_message_type_support_t _SigData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SigData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SigData)() {
  return &_SigData__type_support;
}

#if defined(__cplusplus)
}
#endif
