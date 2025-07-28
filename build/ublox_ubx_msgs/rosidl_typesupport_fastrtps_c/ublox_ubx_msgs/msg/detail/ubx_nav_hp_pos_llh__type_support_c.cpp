// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosLLH.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_llh__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_llh__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_llh__functions.h"
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


using _UBXNavHPPosLLH__ros_msg_type = ublox_ubx_msgs__msg__UBXNavHPPosLLH;

static bool _UBXNavHPPosLLH__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UBXNavHPPosLLH__ros_msg_type * ros_message = static_cast<const _UBXNavHPPosLLH__ros_msg_type *>(untyped_ros_message);
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

  // Field name: invalid_lon
  {
    cdr << (ros_message->invalid_lon ? true : false);
  }

  // Field name: invalid_lat
  {
    cdr << (ros_message->invalid_lat ? true : false);
  }

  // Field name: invalid_height
  {
    cdr << (ros_message->invalid_height ? true : false);
  }

  // Field name: invalid_hmsl
  {
    cdr << (ros_message->invalid_hmsl ? true : false);
  }

  // Field name: invalid_lon_hp
  {
    cdr << (ros_message->invalid_lon_hp ? true : false);
  }

  // Field name: invalid_lat_hp
  {
    cdr << (ros_message->invalid_lat_hp ? true : false);
  }

  // Field name: invalid_height_hp
  {
    cdr << (ros_message->invalid_height_hp ? true : false);
  }

  // Field name: invalid_hmsl_hp
  {
    cdr << (ros_message->invalid_hmsl_hp ? true : false);
  }

  // Field name: itow
  {
    cdr << ros_message->itow;
  }

  // Field name: lon
  {
    cdr << ros_message->lon;
  }

  // Field name: lat
  {
    cdr << ros_message->lat;
  }

  // Field name: height
  {
    cdr << ros_message->height;
  }

  // Field name: hmsl
  {
    cdr << ros_message->hmsl;
  }

  // Field name: lon_hp
  {
    cdr << ros_message->lon_hp;
  }

  // Field name: lat_hp
  {
    cdr << ros_message->lat_hp;
  }

  // Field name: height_hp
  {
    cdr << ros_message->height_hp;
  }

  // Field name: hmsl_hp
  {
    cdr << ros_message->hmsl_hp;
  }

  // Field name: h_acc
  {
    cdr << ros_message->h_acc;
  }

  // Field name: v_acc
  {
    cdr << ros_message->v_acc;
  }

  return true;
}

static bool _UBXNavHPPosLLH__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UBXNavHPPosLLH__ros_msg_type * ros_message = static_cast<_UBXNavHPPosLLH__ros_msg_type *>(untyped_ros_message);
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

  // Field name: invalid_lon
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_lon = tmp ? true : false;
  }

  // Field name: invalid_lat
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_lat = tmp ? true : false;
  }

  // Field name: invalid_height
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_height = tmp ? true : false;
  }

  // Field name: invalid_hmsl
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_hmsl = tmp ? true : false;
  }

  // Field name: invalid_lon_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_lon_hp = tmp ? true : false;
  }

  // Field name: invalid_lat_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_lat_hp = tmp ? true : false;
  }

  // Field name: invalid_height_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_height_hp = tmp ? true : false;
  }

  // Field name: invalid_hmsl_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_hmsl_hp = tmp ? true : false;
  }

  // Field name: itow
  {
    cdr >> ros_message->itow;
  }

  // Field name: lon
  {
    cdr >> ros_message->lon;
  }

  // Field name: lat
  {
    cdr >> ros_message->lat;
  }

  // Field name: height
  {
    cdr >> ros_message->height;
  }

  // Field name: hmsl
  {
    cdr >> ros_message->hmsl;
  }

  // Field name: lon_hp
  {
    cdr >> ros_message->lon_hp;
  }

  // Field name: lat_hp
  {
    cdr >> ros_message->lat_hp;
  }

  // Field name: height_hp
  {
    cdr >> ros_message->height_hp;
  }

  // Field name: hmsl_hp
  {
    cdr >> ros_message->hmsl_hp;
  }

  // Field name: h_acc
  {
    cdr >> ros_message->h_acc;
  }

  // Field name: v_acc
  {
    cdr >> ros_message->v_acc;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__UBXNavHPPosLLH(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UBXNavHPPosLLH__ros_msg_type * ros_message = static_cast<const _UBXNavHPPosLLH__ros_msg_type *>(untyped_ros_message);
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
  // field.name invalid_lon
  {
    size_t item_size = sizeof(ros_message->invalid_lon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_lat
  {
    size_t item_size = sizeof(ros_message->invalid_lat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_height
  {
    size_t item_size = sizeof(ros_message->invalid_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_hmsl
  {
    size_t item_size = sizeof(ros_message->invalid_hmsl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_lon_hp
  {
    size_t item_size = sizeof(ros_message->invalid_lon_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_lat_hp
  {
    size_t item_size = sizeof(ros_message->invalid_lat_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_height_hp
  {
    size_t item_size = sizeof(ros_message->invalid_height_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_hmsl_hp
  {
    size_t item_size = sizeof(ros_message->invalid_hmsl_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name itow
  {
    size_t item_size = sizeof(ros_message->itow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lon
  {
    size_t item_size = sizeof(ros_message->lon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lat
  {
    size_t item_size = sizeof(ros_message->lat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name height
  {
    size_t item_size = sizeof(ros_message->height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hmsl
  {
    size_t item_size = sizeof(ros_message->hmsl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lon_hp
  {
    size_t item_size = sizeof(ros_message->lon_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lat_hp
  {
    size_t item_size = sizeof(ros_message->lat_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name height_hp
  {
    size_t item_size = sizeof(ros_message->height_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hmsl_hp
  {
    size_t item_size = sizeof(ros_message->hmsl_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name h_acc
  {
    size_t item_size = sizeof(ros_message->h_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name v_acc
  {
    size_t item_size = sizeof(ros_message->v_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UBXNavHPPosLLH__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__UBXNavHPPosLLH(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__UBXNavHPPosLLH(
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
  // member: invalid_lon
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_lat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_height
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_hmsl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_lon_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_lat_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_height_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invalid_hmsl_hp
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
  // member: lon
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: lat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: height
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: hmsl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: lon_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: lat_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: height_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: hmsl_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: h_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: v_acc
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
    using DataType = ublox_ubx_msgs__msg__UBXNavHPPosLLH;
    is_plain =
      (
      offsetof(DataType, v_acc) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _UBXNavHPPosLLH__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__UBXNavHPPosLLH(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UBXNavHPPosLLH = {
  "ublox_ubx_msgs::msg",
  "UBXNavHPPosLLH",
  _UBXNavHPPosLLH__cdr_serialize,
  _UBXNavHPPosLLH__cdr_deserialize,
  _UBXNavHPPosLLH__get_serialized_size,
  _UBXNavHPPosLLH__max_serialized_size
};

static rosidl_message_type_support_t _UBXNavHPPosLLH__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UBXNavHPPosLLH,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, UBXNavHPPosLLH)() {
  return &_UBXNavHPPosLLH__type_support;
}

#if defined(__cplusplus)
}
#endif
