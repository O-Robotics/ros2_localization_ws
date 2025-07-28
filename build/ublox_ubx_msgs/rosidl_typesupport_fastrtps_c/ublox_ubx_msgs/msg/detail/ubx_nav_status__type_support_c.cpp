// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavStatus.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__functions.h"
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
#include "ublox_ubx_msgs/msg/detail/carr_soln__functions.h"  // carr_soln
#include "ublox_ubx_msgs/msg/detail/gps_fix__functions.h"  // gps_fix
#include "ublox_ubx_msgs/msg/detail/map_matching__functions.h"  // map_matching
#include "ublox_ubx_msgs/msg/detail/psm_status__functions.h"  // psm
#include "ublox_ubx_msgs/msg/detail/spoof_det__functions.h"  // spoof_det

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
size_t get_serialized_size_ublox_ubx_msgs__msg__CarrSoln(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__CarrSoln(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, CarrSoln)();
size_t get_serialized_size_ublox_ubx_msgs__msg__GpsFix(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__GpsFix(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, GpsFix)();
size_t get_serialized_size_ublox_ubx_msgs__msg__MapMatching(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__MapMatching(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, MapMatching)();
size_t get_serialized_size_ublox_ubx_msgs__msg__PSMStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__PSMStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, PSMStatus)();
size_t get_serialized_size_ublox_ubx_msgs__msg__SpoofDet(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__SpoofDet(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SpoofDet)();


using _UBXNavStatus__ros_msg_type = ublox_ubx_msgs__msg__UBXNavStatus;

static bool _UBXNavStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UBXNavStatus__ros_msg_type * ros_message = static_cast<const _UBXNavStatus__ros_msg_type *>(untyped_ros_message);
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

  // Field name: itow
  {
    cdr << ros_message->itow;
  }

  // Field name: gps_fix
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, GpsFix
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->gps_fix, cdr))
    {
      return false;
    }
  }

  // Field name: gps_fix_ok
  {
    cdr << (ros_message->gps_fix_ok ? true : false);
  }

  // Field name: diff_soln
  {
    cdr << (ros_message->diff_soln ? true : false);
  }

  // Field name: wkn_set
  {
    cdr << (ros_message->wkn_set ? true : false);
  }

  // Field name: tow_set
  {
    cdr << (ros_message->tow_set ? true : false);
  }

  // Field name: diff_corr
  {
    cdr << (ros_message->diff_corr ? true : false);
  }

  // Field name: carr_soln_valid
  {
    cdr << (ros_message->carr_soln_valid ? true : false);
  }

  // Field name: map_matching
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, MapMatching
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->map_matching, cdr))
    {
      return false;
    }
  }

  // Field name: psm
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, PSMStatus
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->psm, cdr))
    {
      return false;
    }
  }

  // Field name: spoof_det
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SpoofDet
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->spoof_det, cdr))
    {
      return false;
    }
  }

  // Field name: carr_soln
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, CarrSoln
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->carr_soln, cdr))
    {
      return false;
    }
  }

  // Field name: ttff
  {
    cdr << ros_message->ttff;
  }

  // Field name: msss
  {
    cdr << ros_message->msss;
  }

  return true;
}

static bool _UBXNavStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UBXNavStatus__ros_msg_type * ros_message = static_cast<_UBXNavStatus__ros_msg_type *>(untyped_ros_message);
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

  // Field name: itow
  {
    cdr >> ros_message->itow;
  }

  // Field name: gps_fix
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, GpsFix
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->gps_fix))
    {
      return false;
    }
  }

  // Field name: gps_fix_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->gps_fix_ok = tmp ? true : false;
  }

  // Field name: diff_soln
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->diff_soln = tmp ? true : false;
  }

  // Field name: wkn_set
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->wkn_set = tmp ? true : false;
  }

  // Field name: tow_set
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->tow_set = tmp ? true : false;
  }

  // Field name: diff_corr
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->diff_corr = tmp ? true : false;
  }

  // Field name: carr_soln_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->carr_soln_valid = tmp ? true : false;
  }

  // Field name: map_matching
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, MapMatching
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->map_matching))
    {
      return false;
    }
  }

  // Field name: psm
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, PSMStatus
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->psm))
    {
      return false;
    }
  }

  // Field name: spoof_det
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SpoofDet
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->spoof_det))
    {
      return false;
    }
  }

  // Field name: carr_soln
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, CarrSoln
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->carr_soln))
    {
      return false;
    }
  }

  // Field name: ttff
  {
    cdr >> ros_message->ttff;
  }

  // Field name: msss
  {
    cdr >> ros_message->msss;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__UBXNavStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UBXNavStatus__ros_msg_type * ros_message = static_cast<const _UBXNavStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name itow
  {
    size_t item_size = sizeof(ros_message->itow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gps_fix

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__GpsFix(
    &(ros_message->gps_fix), current_alignment);
  // field.name gps_fix_ok
  {
    size_t item_size = sizeof(ros_message->gps_fix_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name diff_soln
  {
    size_t item_size = sizeof(ros_message->diff_soln);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name wkn_set
  {
    size_t item_size = sizeof(ros_message->wkn_set);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tow_set
  {
    size_t item_size = sizeof(ros_message->tow_set);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name diff_corr
  {
    size_t item_size = sizeof(ros_message->diff_corr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name carr_soln_valid
  {
    size_t item_size = sizeof(ros_message->carr_soln_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name map_matching

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__MapMatching(
    &(ros_message->map_matching), current_alignment);
  // field.name psm

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__PSMStatus(
    &(ros_message->psm), current_alignment);
  // field.name spoof_det

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__SpoofDet(
    &(ros_message->spoof_det), current_alignment);
  // field.name carr_soln

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__CarrSoln(
    &(ros_message->carr_soln), current_alignment);
  // field.name ttff
  {
    size_t item_size = sizeof(ros_message->ttff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name msss
  {
    size_t item_size = sizeof(ros_message->msss);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UBXNavStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__UBXNavStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__UBXNavStatus(
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
  // member: itow
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gps_fix
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__GpsFix(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: gps_fix_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: diff_soln
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: wkn_set
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: tow_set
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: diff_corr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: carr_soln_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: map_matching
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__MapMatching(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: psm
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__PSMStatus(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: spoof_det
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__SpoofDet(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: carr_soln
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__CarrSoln(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: ttff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: msss
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ublox_ubx_msgs__msg__UBXNavStatus;
    is_plain =
      (
      offsetof(DataType, msss) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _UBXNavStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__UBXNavStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UBXNavStatus = {
  "ublox_ubx_msgs::msg",
  "UBXNavStatus",
  _UBXNavStatus__cdr_serialize,
  _UBXNavStatus__cdr_deserialize,
  _UBXNavStatus__get_serialized_size,
  _UBXNavStatus__max_serialized_size
};

static rosidl_message_type_support_t _UBXNavStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UBXNavStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, UBXNavStatus)() {
  return &_UBXNavStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
