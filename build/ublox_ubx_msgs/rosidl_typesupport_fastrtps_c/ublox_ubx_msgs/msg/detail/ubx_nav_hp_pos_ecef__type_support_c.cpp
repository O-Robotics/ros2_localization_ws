// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosECEF.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__functions.h"
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


using _UBXNavHPPosECEF__ros_msg_type = ublox_ubx_msgs__msg__UBXNavHPPosECEF;

static bool _UBXNavHPPosECEF__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UBXNavHPPosECEF__ros_msg_type * ros_message = static_cast<const _UBXNavHPPosECEF__ros_msg_type *>(untyped_ros_message);
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

  // Field name: version
  {
    cdr << ros_message->version;
  }

  // Field name: itow
  {
    cdr << ros_message->itow;
  }

  // Field name: ecef_x
  {
    cdr << ros_message->ecef_x;
  }

  // Field name: ecef_y
  {
    cdr << ros_message->ecef_y;
  }

  // Field name: ecef_z
  {
    cdr << ros_message->ecef_z;
  }

  // Field name: ecef_x_hp
  {
    cdr << ros_message->ecef_x_hp;
  }

  // Field name: ecef_y_hp
  {
    cdr << ros_message->ecef_y_hp;
  }

  // Field name: ecef_z_hp
  {
    cdr << ros_message->ecef_z_hp;
  }

  // Field name: invalid_ecef_x
  {
    cdr << (ros_message->invalid_ecef_x ? true : false);
  }

  // Field name: invalid_ecef_y
  {
    cdr << (ros_message->invalid_ecef_y ? true : false);
  }

  // Field name: invalid_ecef_z
  {
    cdr << (ros_message->invalid_ecef_z ? true : false);
  }

  // Field name: invalid_ecef_x_hp
  {
    cdr << (ros_message->invalid_ecef_x_hp ? true : false);
  }

  // Field name: invalid_ecef_y_hp
  {
    cdr << (ros_message->invalid_ecef_y_hp ? true : false);
  }

  // Field name: invalid_ecef_z_hp
  {
    cdr << (ros_message->invalid_ecef_z_hp ? true : false);
  }

  // Field name: p_acc
  {
    cdr << ros_message->p_acc;
  }

  return true;
}

static bool _UBXNavHPPosECEF__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UBXNavHPPosECEF__ros_msg_type * ros_message = static_cast<_UBXNavHPPosECEF__ros_msg_type *>(untyped_ros_message);
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

  // Field name: version
  {
    cdr >> ros_message->version;
  }

  // Field name: itow
  {
    cdr >> ros_message->itow;
  }

  // Field name: ecef_x
  {
    cdr >> ros_message->ecef_x;
  }

  // Field name: ecef_y
  {
    cdr >> ros_message->ecef_y;
  }

  // Field name: ecef_z
  {
    cdr >> ros_message->ecef_z;
  }

  // Field name: ecef_x_hp
  {
    cdr >> ros_message->ecef_x_hp;
  }

  // Field name: ecef_y_hp
  {
    cdr >> ros_message->ecef_y_hp;
  }

  // Field name: ecef_z_hp
  {
    cdr >> ros_message->ecef_z_hp;
  }

  // Field name: invalid_ecef_x
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_ecef_x = tmp ? true : false;
  }

  // Field name: invalid_ecef_y
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_ecef_y = tmp ? true : false;
  }

  // Field name: invalid_ecef_z
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_ecef_z = tmp ? true : false;
  }

  // Field name: invalid_ecef_x_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_ecef_x_hp = tmp ? true : false;
  }

  // Field name: invalid_ecef_y_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_ecef_y_hp = tmp ? true : false;
  }

  // Field name: invalid_ecef_z_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_ecef_z_hp = tmp ? true : false;
  }

  // Field name: p_acc
  {
    cdr >> ros_message->p_acc;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__UBXNavHPPosECEF(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UBXNavHPPosECEF__ros_msg_type * ros_message = static_cast<const _UBXNavHPPosECEF__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name version
  {
    size_t item_size = sizeof(ros_message->version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name itow
  {
    size_t item_size = sizeof(ros_message->itow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ecef_x
  {
    size_t item_size = sizeof(ros_message->ecef_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ecef_y
  {
    size_t item_size = sizeof(ros_message->ecef_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ecef_z
  {
    size_t item_size = sizeof(ros_message->ecef_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ecef_x_hp
  {
    size_t item_size = sizeof(ros_message->ecef_x_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ecef_y_hp
  {
    size_t item_size = sizeof(ros_message->ecef_y_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ecef_z_hp
  {
    size_t item_size = sizeof(ros_message->ecef_z_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_ecef_x
  {
    size_t item_size = sizeof(ros_message->invalid_ecef_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_ecef_y
  {
    size_t item_size = sizeof(ros_message->invalid_ecef_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_ecef_z
  {
    size_t item_size = sizeof(ros_message->invalid_ecef_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_ecef_x_hp
  {
    size_t item_size = sizeof(ros_message->invalid_ecef_x_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_ecef_y_hp
  {
    size_t item_size = sizeof(ros_message->invalid_ecef_y_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_ecef_z_hp
  {
    size_t item_size = sizeof(ros_message->invalid_ecef_z_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name p_acc
  {
    size_t item_size = sizeof(ros_message->p_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UBXNavHPPosECEF__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__UBXNavHPPosECEF(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__UBXNavHPPosECEF(
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
  // member: version
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: itow
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ecef_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ecef_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ecef_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ecef_x_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ecef_y_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ecef_z_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_ecef_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_ecef_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_ecef_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_ecef_x_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_ecef_y_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_ecef_z_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: p_acc
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
    using DataType = ublox_ubx_msgs__msg__UBXNavHPPosECEF;
    is_plain =
      (
      offsetof(DataType, p_acc) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _UBXNavHPPosECEF__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__UBXNavHPPosECEF(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UBXNavHPPosECEF = {
  "ublox_ubx_msgs::msg",
  "UBXNavHPPosECEF",
  _UBXNavHPPosECEF__cdr_serialize,
  _UBXNavHPPosECEF__cdr_deserialize,
  _UBXNavHPPosECEF__get_serialized_size,
  _UBXNavHPPosECEF__max_serialized_size
};

static rosidl_message_type_support_t _UBXNavHPPosECEF__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UBXNavHPPosECEF,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, UBXNavHPPosECEF)() {
  return &_UBXNavHPPosECEF__type_support;
}

#if defined(__cplusplus)
}
#endif
