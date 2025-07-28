// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavRelPosNED.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__functions.h"
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


using _UBXNavRelPosNED__ros_msg_type = ublox_ubx_msgs__msg__UBXNavRelPosNED;

static bool _UBXNavRelPosNED__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UBXNavRelPosNED__ros_msg_type * ros_message = static_cast<const _UBXNavRelPosNED__ros_msg_type *>(untyped_ros_message);
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

  // Field name: ref_station_id
  {
    cdr << ros_message->ref_station_id;
  }

  // Field name: itow
  {
    cdr << ros_message->itow;
  }

  // Field name: rel_pos_n
  {
    cdr << ros_message->rel_pos_n;
  }

  // Field name: rel_pos_e
  {
    cdr << ros_message->rel_pos_e;
  }

  // Field name: rel_pos_d
  {
    cdr << ros_message->rel_pos_d;
  }

  // Field name: rel_pos_length
  {
    cdr << ros_message->rel_pos_length;
  }

  // Field name: rel_pos_heading
  {
    cdr << ros_message->rel_pos_heading;
  }

  // Field name: rel_pos_hp_n
  {
    cdr << ros_message->rel_pos_hp_n;
  }

  // Field name: rel_pos_hp_e
  {
    cdr << ros_message->rel_pos_hp_e;
  }

  // Field name: rel_pos_hp_d
  {
    cdr << ros_message->rel_pos_hp_d;
  }

  // Field name: rel_pos_hp_length
  {
    cdr << ros_message->rel_pos_hp_length;
  }

  // Field name: acc_n
  {
    cdr << ros_message->acc_n;
  }

  // Field name: acc_e
  {
    cdr << ros_message->acc_e;
  }

  // Field name: acc_d
  {
    cdr << ros_message->acc_d;
  }

  // Field name: acc_length
  {
    cdr << ros_message->acc_length;
  }

  // Field name: acc_heading
  {
    cdr << ros_message->acc_heading;
  }

  // Field name: gnss_fix_ok
  {
    cdr << (ros_message->gnss_fix_ok ? true : false);
  }

  // Field name: diff_soln
  {
    cdr << (ros_message->diff_soln ? true : false);
  }

  // Field name: rel_pos_valid
  {
    cdr << (ros_message->rel_pos_valid ? true : false);
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

  // Field name: is_moving
  {
    cdr << (ros_message->is_moving ? true : false);
  }

  // Field name: ref_pos_miss
  {
    cdr << (ros_message->ref_pos_miss ? true : false);
  }

  // Field name: ref_obs_miss
  {
    cdr << (ros_message->ref_obs_miss ? true : false);
  }

  // Field name: rel_pos_heading_valid
  {
    cdr << (ros_message->rel_pos_heading_valid ? true : false);
  }

  // Field name: rel_pos_normalized
  {
    cdr << (ros_message->rel_pos_normalized ? true : false);
  }

  return true;
}

static bool _UBXNavRelPosNED__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UBXNavRelPosNED__ros_msg_type * ros_message = static_cast<_UBXNavRelPosNED__ros_msg_type *>(untyped_ros_message);
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

  // Field name: ref_station_id
  {
    cdr >> ros_message->ref_station_id;
  }

  // Field name: itow
  {
    cdr >> ros_message->itow;
  }

  // Field name: rel_pos_n
  {
    cdr >> ros_message->rel_pos_n;
  }

  // Field name: rel_pos_e
  {
    cdr >> ros_message->rel_pos_e;
  }

  // Field name: rel_pos_d
  {
    cdr >> ros_message->rel_pos_d;
  }

  // Field name: rel_pos_length
  {
    cdr >> ros_message->rel_pos_length;
  }

  // Field name: rel_pos_heading
  {
    cdr >> ros_message->rel_pos_heading;
  }

  // Field name: rel_pos_hp_n
  {
    cdr >> ros_message->rel_pos_hp_n;
  }

  // Field name: rel_pos_hp_e
  {
    cdr >> ros_message->rel_pos_hp_e;
  }

  // Field name: rel_pos_hp_d
  {
    cdr >> ros_message->rel_pos_hp_d;
  }

  // Field name: rel_pos_hp_length
  {
    cdr >> ros_message->rel_pos_hp_length;
  }

  // Field name: acc_n
  {
    cdr >> ros_message->acc_n;
  }

  // Field name: acc_e
  {
    cdr >> ros_message->acc_e;
  }

  // Field name: acc_d
  {
    cdr >> ros_message->acc_d;
  }

  // Field name: acc_length
  {
    cdr >> ros_message->acc_length;
  }

  // Field name: acc_heading
  {
    cdr >> ros_message->acc_heading;
  }

  // Field name: gnss_fix_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->gnss_fix_ok = tmp ? true : false;
  }

  // Field name: diff_soln
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->diff_soln = tmp ? true : false;
  }

  // Field name: rel_pos_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->rel_pos_valid = tmp ? true : false;
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

  // Field name: is_moving
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_moving = tmp ? true : false;
  }

  // Field name: ref_pos_miss
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->ref_pos_miss = tmp ? true : false;
  }

  // Field name: ref_obs_miss
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->ref_obs_miss = tmp ? true : false;
  }

  // Field name: rel_pos_heading_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->rel_pos_heading_valid = tmp ? true : false;
  }

  // Field name: rel_pos_normalized
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->rel_pos_normalized = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__UBXNavRelPosNED(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UBXNavRelPosNED__ros_msg_type * ros_message = static_cast<const _UBXNavRelPosNED__ros_msg_type *>(untyped_ros_message);
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
  // field.name ref_station_id
  {
    size_t item_size = sizeof(ros_message->ref_station_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name itow
  {
    size_t item_size = sizeof(ros_message->itow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_n
  {
    size_t item_size = sizeof(ros_message->rel_pos_n);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_e
  {
    size_t item_size = sizeof(ros_message->rel_pos_e);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_d
  {
    size_t item_size = sizeof(ros_message->rel_pos_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_length
  {
    size_t item_size = sizeof(ros_message->rel_pos_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_heading
  {
    size_t item_size = sizeof(ros_message->rel_pos_heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_hp_n
  {
    size_t item_size = sizeof(ros_message->rel_pos_hp_n);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_hp_e
  {
    size_t item_size = sizeof(ros_message->rel_pos_hp_e);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_hp_d
  {
    size_t item_size = sizeof(ros_message->rel_pos_hp_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_hp_length
  {
    size_t item_size = sizeof(ros_message->rel_pos_hp_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name acc_n
  {
    size_t item_size = sizeof(ros_message->acc_n);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name acc_e
  {
    size_t item_size = sizeof(ros_message->acc_e);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name acc_d
  {
    size_t item_size = sizeof(ros_message->acc_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name acc_length
  {
    size_t item_size = sizeof(ros_message->acc_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name acc_heading
  {
    size_t item_size = sizeof(ros_message->acc_heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gnss_fix_ok
  {
    size_t item_size = sizeof(ros_message->gnss_fix_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name diff_soln
  {
    size_t item_size = sizeof(ros_message->diff_soln);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_valid
  {
    size_t item_size = sizeof(ros_message->rel_pos_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name carr_soln

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__CarrSoln(
    &(ros_message->carr_soln), current_alignment);
  // field.name is_moving
  {
    size_t item_size = sizeof(ros_message->is_moving);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ref_pos_miss
  {
    size_t item_size = sizeof(ros_message->ref_pos_miss);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ref_obs_miss
  {
    size_t item_size = sizeof(ros_message->ref_obs_miss);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_heading_valid
  {
    size_t item_size = sizeof(ros_message->rel_pos_heading_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rel_pos_normalized
  {
    size_t item_size = sizeof(ros_message->rel_pos_normalized);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UBXNavRelPosNED__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__UBXNavRelPosNED(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__UBXNavRelPosNED(
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
  // member: ref_station_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: itow
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rel_pos_n
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rel_pos_e
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rel_pos_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rel_pos_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rel_pos_heading
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rel_pos_hp_n
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rel_pos_hp_e
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rel_pos_hp_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rel_pos_hp_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: acc_n
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: acc_e
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: acc_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: acc_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: acc_heading
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gnss_fix_ok
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
  // member: rel_pos_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
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
  // member: is_moving
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ref_pos_miss
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ref_obs_miss
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rel_pos_heading_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rel_pos_normalized
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
    using DataType = ublox_ubx_msgs__msg__UBXNavRelPosNED;
    is_plain =
      (
      offsetof(DataType, rel_pos_normalized) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _UBXNavRelPosNED__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__UBXNavRelPosNED(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UBXNavRelPosNED = {
  "ublox_ubx_msgs::msg",
  "UBXNavRelPosNED",
  _UBXNavRelPosNED__cdr_serialize,
  _UBXNavRelPosNED__cdr_deserialize,
  _UBXNavRelPosNED__get_serialized_size,
  _UBXNavRelPosNED__max_serialized_size
};

static rosidl_message_type_support_t _UBXNavRelPosNED__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UBXNavRelPosNED,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, UBXNavRelPosNED)() {
  return &_UBXNavRelPosNED__type_support;
}

#if defined(__cplusplus)
}
#endif
