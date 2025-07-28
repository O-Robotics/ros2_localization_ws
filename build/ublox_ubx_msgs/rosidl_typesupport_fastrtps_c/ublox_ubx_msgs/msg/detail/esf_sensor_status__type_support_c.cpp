// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/ESFSensorStatus.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__struct.h"
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__functions.h"
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


using _ESFSensorStatus__ros_msg_type = ublox_ubx_msgs__msg__ESFSensorStatus;

static bool _ESFSensorStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ESFSensorStatus__ros_msg_type * ros_message = static_cast<const _ESFSensorStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: sensor_data_type
  {
    cdr << ros_message->sensor_data_type;
  }

  // Field name: used
  {
    cdr << (ros_message->used ? true : false);
  }

  // Field name: ready
  {
    cdr << (ros_message->ready ? true : false);
  }

  // Field name: calib_status
  {
    cdr << ros_message->calib_status;
  }

  // Field name: time_status
  {
    cdr << ros_message->time_status;
  }

  // Field name: freq
  {
    cdr << ros_message->freq;
  }

  // Field name: fault_bad_meas
  {
    cdr << (ros_message->fault_bad_meas ? true : false);
  }

  // Field name: fault_bad_ttag
  {
    cdr << (ros_message->fault_bad_ttag ? true : false);
  }

  // Field name: fault_missing_meas
  {
    cdr << (ros_message->fault_missing_meas ? true : false);
  }

  // Field name: fault_noisy_meas
  {
    cdr << (ros_message->fault_noisy_meas ? true : false);
  }

  return true;
}

static bool _ESFSensorStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ESFSensorStatus__ros_msg_type * ros_message = static_cast<_ESFSensorStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: sensor_data_type
  {
    cdr >> ros_message->sensor_data_type;
  }

  // Field name: used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->used = tmp ? true : false;
  }

  // Field name: ready
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->ready = tmp ? true : false;
  }

  // Field name: calib_status
  {
    cdr >> ros_message->calib_status;
  }

  // Field name: time_status
  {
    cdr >> ros_message->time_status;
  }

  // Field name: freq
  {
    cdr >> ros_message->freq;
  }

  // Field name: fault_bad_meas
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fault_bad_meas = tmp ? true : false;
  }

  // Field name: fault_bad_ttag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fault_bad_ttag = tmp ? true : false;
  }

  // Field name: fault_missing_meas
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fault_missing_meas = tmp ? true : false;
  }

  // Field name: fault_noisy_meas
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fault_noisy_meas = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__ESFSensorStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ESFSensorStatus__ros_msg_type * ros_message = static_cast<const _ESFSensorStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name sensor_data_type
  {
    size_t item_size = sizeof(ros_message->sensor_data_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name used
  {
    size_t item_size = sizeof(ros_message->used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ready
  {
    size_t item_size = sizeof(ros_message->ready);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name calib_status
  {
    size_t item_size = sizeof(ros_message->calib_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name time_status
  {
    size_t item_size = sizeof(ros_message->time_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name freq
  {
    size_t item_size = sizeof(ros_message->freq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fault_bad_meas
  {
    size_t item_size = sizeof(ros_message->fault_bad_meas);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fault_bad_ttag
  {
    size_t item_size = sizeof(ros_message->fault_bad_ttag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fault_missing_meas
  {
    size_t item_size = sizeof(ros_message->fault_missing_meas);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fault_noisy_meas
  {
    size_t item_size = sizeof(ros_message->fault_noisy_meas);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ESFSensorStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__ESFSensorStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__ESFSensorStatus(
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

  // member: sensor_data_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ready
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: calib_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: time_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: freq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fault_bad_meas
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fault_bad_ttag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fault_missing_meas
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fault_noisy_meas
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
    using DataType = ublox_ubx_msgs__msg__ESFSensorStatus;
    is_plain =
      (
      offsetof(DataType, fault_noisy_meas) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ESFSensorStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__ESFSensorStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ESFSensorStatus = {
  "ublox_ubx_msgs::msg",
  "ESFSensorStatus",
  _ESFSensorStatus__cdr_serialize,
  _ESFSensorStatus__cdr_deserialize,
  _ESFSensorStatus__get_serialized_size,
  _ESFSensorStatus__max_serialized_size
};

static rosidl_message_type_support_t _ESFSensorStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ESFSensorStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, ESFSensorStatus)() {
  return &_ESFSensorStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
