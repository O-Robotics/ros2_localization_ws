// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmRawx.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__functions.h"
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

#include "std_msgs/msg/detail/header__functions.h"  // header
#include "ublox_ubx_msgs/msg/detail/rawx_data__functions.h"  // rawx_data
#include "ublox_ubx_msgs/msg/detail/rec_stat__functions.h"  // rec_stat

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ublox_ubx_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ublox_ubx_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ublox_ubx_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();
size_t get_serialized_size_ublox_ubx_msgs__msg__RawxData(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__RawxData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, RawxData)();
size_t get_serialized_size_ublox_ubx_msgs__msg__RecStat(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__RecStat(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, RecStat)();


using _UBXRxmRawx__ros_msg_type = ublox_ubx_msgs__msg__UBXRxmRawx;

static bool _UBXRxmRawx__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UBXRxmRawx__ros_msg_type * ros_message = static_cast<const _UBXRxmRawx__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: rcv_tow
  {
    cdr << ros_message->rcv_tow;
  }

  // Field name: week
  {
    cdr << ros_message->week;
  }

  // Field name: leap_s
  {
    cdr << ros_message->leap_s;
  }

  // Field name: num_meas
  {
    cdr << ros_message->num_meas;
  }

  // Field name: rec_stat
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, RecStat
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->rec_stat, cdr))
    {
      return false;
    }
  }

  // Field name: version
  {
    cdr << ros_message->version;
  }

  // Field name: rawx_data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, RawxData
      )()->data);
    size_t size = ros_message->rawx_data.size;
    auto array_ptr = ros_message->rawx_data.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _UBXRxmRawx__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UBXRxmRawx__ros_msg_type * ros_message = static_cast<_UBXRxmRawx__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: rcv_tow
  {
    cdr >> ros_message->rcv_tow;
  }

  // Field name: week
  {
    cdr >> ros_message->week;
  }

  // Field name: leap_s
  {
    cdr >> ros_message->leap_s;
  }

  // Field name: num_meas
  {
    cdr >> ros_message->num_meas;
  }

  // Field name: rec_stat
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, RecStat
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->rec_stat))
    {
      return false;
    }
  }

  // Field name: version
  {
    cdr >> ros_message->version;
  }

  // Field name: rawx_data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, RawxData
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->rawx_data.data) {
      ublox_ubx_msgs__msg__RawxData__Sequence__fini(&ros_message->rawx_data);
    }
    if (!ublox_ubx_msgs__msg__RawxData__Sequence__init(&ros_message->rawx_data, size)) {
      fprintf(stderr, "failed to create array for field 'rawx_data'");
      return false;
    }
    auto array_ptr = ros_message->rawx_data.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__UBXRxmRawx(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UBXRxmRawx__ros_msg_type * ros_message = static_cast<const _UBXRxmRawx__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name rcv_tow
  {
    size_t item_size = sizeof(ros_message->rcv_tow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name week
  {
    size_t item_size = sizeof(ros_message->week);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name leap_s
  {
    size_t item_size = sizeof(ros_message->leap_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name num_meas
  {
    size_t item_size = sizeof(ros_message->num_meas);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rec_stat

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__RecStat(
    &(ros_message->rec_stat), current_alignment);
  // field.name version
  {
    size_t item_size = sizeof(ros_message->version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rawx_data
  {
    size_t array_size = ros_message->rawx_data.size;
    auto array_ptr = ros_message->rawx_data.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_ublox_ubx_msgs__msg__RawxData(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UBXRxmRawx__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__UBXRxmRawx(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__UBXRxmRawx(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: rcv_tow
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: week
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: leap_s
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: num_meas
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rec_stat
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__RecStat(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: version
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rawx_data
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__RawxData(
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
    using DataType = ublox_ubx_msgs__msg__UBXRxmRawx;
    is_plain =
      (
      offsetof(DataType, rawx_data) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _UBXRxmRawx__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__UBXRxmRawx(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UBXRxmRawx = {
  "ublox_ubx_msgs::msg",
  "UBXRxmRawx",
  _UBXRxmRawx__cdr_serialize,
  _UBXRxmRawx__cdr_deserialize,
  _UBXRxmRawx__get_serialized_size,
  _UBXRxmRawx__max_serialized_size
};

static rosidl_message_type_support_t _UBXRxmRawx__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UBXRxmRawx,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, UBXRxmRawx)() {
  return &_UBXRxmRawx__type_support;
}

#if defined(__cplusplus)
}
#endif
