// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPVT.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__functions.h"
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
#include "ublox_ubx_msgs/msg/detail/psmpvt__functions.h"  // psm

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
size_t get_serialized_size_ublox_ubx_msgs__msg__PSMPVT(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ublox_ubx_msgs__msg__PSMPVT(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, PSMPVT)();


using _UBXNavPVT__ros_msg_type = ublox_ubx_msgs__msg__UBXNavPVT;

static bool _UBXNavPVT__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UBXNavPVT__ros_msg_type * ros_message = static_cast<const _UBXNavPVT__ros_msg_type *>(untyped_ros_message);
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

  // Field name: year
  {
    cdr << ros_message->year;
  }

  // Field name: month
  {
    cdr << ros_message->month;
  }

  // Field name: day
  {
    cdr << ros_message->day;
  }

  // Field name: hour
  {
    cdr << ros_message->hour;
  }

  // Field name: min
  {
    cdr << ros_message->min;
  }

  // Field name: sec
  {
    cdr << ros_message->sec;
  }

  // Field name: valid_date
  {
    cdr << (ros_message->valid_date ? true : false);
  }

  // Field name: valid_time
  {
    cdr << (ros_message->valid_time ? true : false);
  }

  // Field name: fully_resolved
  {
    cdr << (ros_message->fully_resolved ? true : false);
  }

  // Field name: valid_mag
  {
    cdr << (ros_message->valid_mag ? true : false);
  }

  // Field name: t_acc
  {
    cdr << ros_message->t_acc;
  }

  // Field name: nano
  {
    cdr << ros_message->nano;
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

  // Field name: gnss_fix_ok
  {
    cdr << (ros_message->gnss_fix_ok ? true : false);
  }

  // Field name: diff_soln
  {
    cdr << (ros_message->diff_soln ? true : false);
  }

  // Field name: psm
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, PSMPVT
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->psm, cdr))
    {
      return false;
    }
  }

  // Field name: head_veh_valid
  {
    cdr << (ros_message->head_veh_valid ? true : false);
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

  // Field name: confirmed_avail
  {
    cdr << (ros_message->confirmed_avail ? true : false);
  }

  // Field name: confirmed_date
  {
    cdr << (ros_message->confirmed_date ? true : false);
  }

  // Field name: confirmed_time
  {
    cdr << (ros_message->confirmed_time ? true : false);
  }

  // Field name: num_sv
  {
    cdr << ros_message->num_sv;
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

  // Field name: h_acc
  {
    cdr << ros_message->h_acc;
  }

  // Field name: v_acc
  {
    cdr << ros_message->v_acc;
  }

  // Field name: vel_n
  {
    cdr << ros_message->vel_n;
  }

  // Field name: vel_e
  {
    cdr << ros_message->vel_e;
  }

  // Field name: vel_d
  {
    cdr << ros_message->vel_d;
  }

  // Field name: g_speed
  {
    cdr << ros_message->g_speed;
  }

  // Field name: head_mot
  {
    cdr << ros_message->head_mot;
  }

  // Field name: s_acc
  {
    cdr << ros_message->s_acc;
  }

  // Field name: head_acc
  {
    cdr << ros_message->head_acc;
  }

  // Field name: p_dop
  {
    cdr << ros_message->p_dop;
  }

  // Field name: invalid_llh
  {
    cdr << (ros_message->invalid_llh ? true : false);
  }

  // Field name: head_veh
  {
    cdr << ros_message->head_veh;
  }

  // Field name: mag_dec
  {
    cdr << ros_message->mag_dec;
  }

  // Field name: mag_acc
  {
    cdr << ros_message->mag_acc;
  }

  return true;
}

static bool _UBXNavPVT__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UBXNavPVT__ros_msg_type * ros_message = static_cast<_UBXNavPVT__ros_msg_type *>(untyped_ros_message);
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

  // Field name: year
  {
    cdr >> ros_message->year;
  }

  // Field name: month
  {
    cdr >> ros_message->month;
  }

  // Field name: day
  {
    cdr >> ros_message->day;
  }

  // Field name: hour
  {
    cdr >> ros_message->hour;
  }

  // Field name: min
  {
    cdr >> ros_message->min;
  }

  // Field name: sec
  {
    cdr >> ros_message->sec;
  }

  // Field name: valid_date
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->valid_date = tmp ? true : false;
  }

  // Field name: valid_time
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->valid_time = tmp ? true : false;
  }

  // Field name: fully_resolved
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fully_resolved = tmp ? true : false;
  }

  // Field name: valid_mag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->valid_mag = tmp ? true : false;
  }

  // Field name: t_acc
  {
    cdr >> ros_message->t_acc;
  }

  // Field name: nano
  {
    cdr >> ros_message->nano;
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

  // Field name: psm
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, PSMPVT
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->psm))
    {
      return false;
    }
  }

  // Field name: head_veh_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->head_veh_valid = tmp ? true : false;
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

  // Field name: confirmed_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->confirmed_avail = tmp ? true : false;
  }

  // Field name: confirmed_date
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->confirmed_date = tmp ? true : false;
  }

  // Field name: confirmed_time
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->confirmed_time = tmp ? true : false;
  }

  // Field name: num_sv
  {
    cdr >> ros_message->num_sv;
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

  // Field name: h_acc
  {
    cdr >> ros_message->h_acc;
  }

  // Field name: v_acc
  {
    cdr >> ros_message->v_acc;
  }

  // Field name: vel_n
  {
    cdr >> ros_message->vel_n;
  }

  // Field name: vel_e
  {
    cdr >> ros_message->vel_e;
  }

  // Field name: vel_d
  {
    cdr >> ros_message->vel_d;
  }

  // Field name: g_speed
  {
    cdr >> ros_message->g_speed;
  }

  // Field name: head_mot
  {
    cdr >> ros_message->head_mot;
  }

  // Field name: s_acc
  {
    cdr >> ros_message->s_acc;
  }

  // Field name: head_acc
  {
    cdr >> ros_message->head_acc;
  }

  // Field name: p_dop
  {
    cdr >> ros_message->p_dop;
  }

  // Field name: invalid_llh
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invalid_llh = tmp ? true : false;
  }

  // Field name: head_veh
  {
    cdr >> ros_message->head_veh;
  }

  // Field name: mag_dec
  {
    cdr >> ros_message->mag_dec;
  }

  // Field name: mag_acc
  {
    cdr >> ros_message->mag_acc;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__UBXNavPVT(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UBXNavPVT__ros_msg_type * ros_message = static_cast<const _UBXNavPVT__ros_msg_type *>(untyped_ros_message);
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
  // field.name year
  {
    size_t item_size = sizeof(ros_message->year);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name month
  {
    size_t item_size = sizeof(ros_message->month);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name day
  {
    size_t item_size = sizeof(ros_message->day);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hour
  {
    size_t item_size = sizeof(ros_message->hour);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name min
  {
    size_t item_size = sizeof(ros_message->min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sec
  {
    size_t item_size = sizeof(ros_message->sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name valid_date
  {
    size_t item_size = sizeof(ros_message->valid_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name valid_time
  {
    size_t item_size = sizeof(ros_message->valid_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fully_resolved
  {
    size_t item_size = sizeof(ros_message->fully_resolved);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name valid_mag
  {
    size_t item_size = sizeof(ros_message->valid_mag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name t_acc
  {
    size_t item_size = sizeof(ros_message->t_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name nano
  {
    size_t item_size = sizeof(ros_message->nano);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gps_fix

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__GpsFix(
    &(ros_message->gps_fix), current_alignment);
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
  // field.name psm

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__PSMPVT(
    &(ros_message->psm), current_alignment);
  // field.name head_veh_valid
  {
    size_t item_size = sizeof(ros_message->head_veh_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name carr_soln

  current_alignment += get_serialized_size_ublox_ubx_msgs__msg__CarrSoln(
    &(ros_message->carr_soln), current_alignment);
  // field.name confirmed_avail
  {
    size_t item_size = sizeof(ros_message->confirmed_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name confirmed_date
  {
    size_t item_size = sizeof(ros_message->confirmed_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name confirmed_time
  {
    size_t item_size = sizeof(ros_message->confirmed_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name num_sv
  {
    size_t item_size = sizeof(ros_message->num_sv);
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
  // field.name vel_n
  {
    size_t item_size = sizeof(ros_message->vel_n);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vel_e
  {
    size_t item_size = sizeof(ros_message->vel_e);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vel_d
  {
    size_t item_size = sizeof(ros_message->vel_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name g_speed
  {
    size_t item_size = sizeof(ros_message->g_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name head_mot
  {
    size_t item_size = sizeof(ros_message->head_mot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name s_acc
  {
    size_t item_size = sizeof(ros_message->s_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name head_acc
  {
    size_t item_size = sizeof(ros_message->head_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name p_dop
  {
    size_t item_size = sizeof(ros_message->p_dop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invalid_llh
  {
    size_t item_size = sizeof(ros_message->invalid_llh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name head_veh
  {
    size_t item_size = sizeof(ros_message->head_veh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mag_dec
  {
    size_t item_size = sizeof(ros_message->mag_dec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mag_acc
  {
    size_t item_size = sizeof(ros_message->mag_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UBXNavPVT__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__UBXNavPVT(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__UBXNavPVT(
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
  // member: year
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: month
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: day
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: hour
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: valid_date
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: valid_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fully_resolved
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: valid_mag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: t_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: nano
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
  // member: psm
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ublox_ubx_msgs__msg__PSMPVT(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: head_veh_valid
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
  // member: confirmed_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: confirmed_date
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: confirmed_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: num_sv
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
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
  // member: vel_n
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vel_e
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vel_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: g_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: head_mot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: s_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: head_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: p_dop
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: invalid_llh
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: head_veh
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: mag_dec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: mag_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ublox_ubx_msgs__msg__UBXNavPVT;
    is_plain =
      (
      offsetof(DataType, mag_acc) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _UBXNavPVT__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__UBXNavPVT(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UBXNavPVT = {
  "ublox_ubx_msgs::msg",
  "UBXNavPVT",
  _UBXNavPVT__cdr_serialize,
  _UBXNavPVT__cdr_deserialize,
  _UBXNavPVT__get_serialized_size,
  _UBXNavPVT__max_serialized_size
};

static rosidl_message_type_support_t _UBXNavPVT__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UBXNavPVT,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, UBXNavPVT)() {
  return &_UBXNavPVT__type_support;
}

#if defined(__cplusplus)
}
#endif
