// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/MeasxData.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/measx_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/measx_data__struct.h"
#include "ublox_ubx_msgs/msg/detail/measx_data__functions.h"
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


using _MeasxData__ros_msg_type = ublox_ubx_msgs__msg__MeasxData;

static bool _MeasxData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MeasxData__ros_msg_type * ros_message = static_cast<const _MeasxData__ros_msg_type *>(untyped_ros_message);
  // Field name: gnss_id
  {
    cdr << ros_message->gnss_id;
  }

  // Field name: sv_id
  {
    cdr << ros_message->sv_id;
  }

  // Field name: c_no
  {
    cdr << ros_message->c_no;
  }

  // Field name: mpath_indic
  {
    cdr << ros_message->mpath_indic;
  }

  // Field name: doppler_ms
  {
    cdr << ros_message->doppler_ms;
  }

  // Field name: doppler_hz
  {
    cdr << ros_message->doppler_hz;
  }

  // Field name: whole_chips
  {
    cdr << ros_message->whole_chips;
  }

  // Field name: frac_chips
  {
    cdr << ros_message->frac_chips;
  }

  // Field name: code_phase
  {
    cdr << ros_message->code_phase;
  }

  // Field name: int_code_phase
  {
    cdr << ros_message->int_code_phase;
  }

  // Field name: pseu_range_rms_err
  {
    cdr << ros_message->pseu_range_rms_err;
  }

  return true;
}

static bool _MeasxData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MeasxData__ros_msg_type * ros_message = static_cast<_MeasxData__ros_msg_type *>(untyped_ros_message);
  // Field name: gnss_id
  {
    cdr >> ros_message->gnss_id;
  }

  // Field name: sv_id
  {
    cdr >> ros_message->sv_id;
  }

  // Field name: c_no
  {
    cdr >> ros_message->c_no;
  }

  // Field name: mpath_indic
  {
    cdr >> ros_message->mpath_indic;
  }

  // Field name: doppler_ms
  {
    cdr >> ros_message->doppler_ms;
  }

  // Field name: doppler_hz
  {
    cdr >> ros_message->doppler_hz;
  }

  // Field name: whole_chips
  {
    cdr >> ros_message->whole_chips;
  }

  // Field name: frac_chips
  {
    cdr >> ros_message->frac_chips;
  }

  // Field name: code_phase
  {
    cdr >> ros_message->code_phase;
  }

  // Field name: int_code_phase
  {
    cdr >> ros_message->int_code_phase;
  }

  // Field name: pseu_range_rms_err
  {
    cdr >> ros_message->pseu_range_rms_err;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__MeasxData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MeasxData__ros_msg_type * ros_message = static_cast<const _MeasxData__ros_msg_type *>(untyped_ros_message);
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
  // field.name c_no
  {
    size_t item_size = sizeof(ros_message->c_no);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mpath_indic
  {
    size_t item_size = sizeof(ros_message->mpath_indic);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name doppler_ms
  {
    size_t item_size = sizeof(ros_message->doppler_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name doppler_hz
  {
    size_t item_size = sizeof(ros_message->doppler_hz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name whole_chips
  {
    size_t item_size = sizeof(ros_message->whole_chips);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name frac_chips
  {
    size_t item_size = sizeof(ros_message->frac_chips);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name code_phase
  {
    size_t item_size = sizeof(ros_message->code_phase);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name int_code_phase
  {
    size_t item_size = sizeof(ros_message->int_code_phase);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pseu_range_rms_err
  {
    size_t item_size = sizeof(ros_message->pseu_range_rms_err);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MeasxData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__MeasxData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__MeasxData(
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
  // member: c_no
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: mpath_indic
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: doppler_ms
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: doppler_hz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: whole_chips
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: frac_chips
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: code_phase
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: int_code_phase
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pseu_range_rms_err
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
    using DataType = ublox_ubx_msgs__msg__MeasxData;
    is_plain =
      (
      offsetof(DataType, pseu_range_rms_err) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MeasxData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__MeasxData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MeasxData = {
  "ublox_ubx_msgs::msg",
  "MeasxData",
  _MeasxData__cdr_serialize,
  _MeasxData__cdr_deserialize,
  _MeasxData__get_serialized_size,
  _MeasxData__max_serialized_size
};

static rosidl_message_type_support_t _MeasxData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MeasxData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, MeasxData)() {
  return &_MeasxData__type_support;
}

#if defined(__cplusplus)
}
#endif
