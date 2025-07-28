// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_msgs:msg/SatFlags.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sat_flags__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_msgs/msg/detail/sat_flags__struct.h"
#include "ublox_ubx_msgs/msg/detail/sat_flags__functions.h"
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


using _SatFlags__ros_msg_type = ublox_ubx_msgs__msg__SatFlags;

static bool _SatFlags__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SatFlags__ros_msg_type * ros_message = static_cast<const _SatFlags__ros_msg_type *>(untyped_ros_message);
  // Field name: quality_ind
  {
    cdr << ros_message->quality_ind;
  }

  // Field name: sv_used
  {
    cdr << (ros_message->sv_used ? true : false);
  }

  // Field name: health
  {
    cdr << ros_message->health;
  }

  // Field name: diff_corr
  {
    cdr << (ros_message->diff_corr ? true : false);
  }

  // Field name: smoothed
  {
    cdr << (ros_message->smoothed ? true : false);
  }

  // Field name: orbit_source
  {
    cdr << ros_message->orbit_source;
  }

  // Field name: eph_avail
  {
    cdr << (ros_message->eph_avail ? true : false);
  }

  // Field name: alm_avail
  {
    cdr << (ros_message->alm_avail ? true : false);
  }

  // Field name: ano_avail
  {
    cdr << (ros_message->ano_avail ? true : false);
  }

  // Field name: aop_avail
  {
    cdr << (ros_message->aop_avail ? true : false);
  }

  // Field name: sbas_corr_used
  {
    cdr << (ros_message->sbas_corr_used ? true : false);
  }

  // Field name: rtcm_corr_used
  {
    cdr << (ros_message->rtcm_corr_used ? true : false);
  }

  // Field name: slas_corr_used
  {
    cdr << (ros_message->slas_corr_used ? true : false);
  }

  // Field name: spartn_corr_used
  {
    cdr << (ros_message->spartn_corr_used ? true : false);
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

  // Field name: clas_corr_used
  {
    cdr << (ros_message->clas_corr_used ? true : false);
  }

  return true;
}

static bool _SatFlags__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SatFlags__ros_msg_type * ros_message = static_cast<_SatFlags__ros_msg_type *>(untyped_ros_message);
  // Field name: quality_ind
  {
    cdr >> ros_message->quality_ind;
  }

  // Field name: sv_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->sv_used = tmp ? true : false;
  }

  // Field name: health
  {
    cdr >> ros_message->health;
  }

  // Field name: diff_corr
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->diff_corr = tmp ? true : false;
  }

  // Field name: smoothed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->smoothed = tmp ? true : false;
  }

  // Field name: orbit_source
  {
    cdr >> ros_message->orbit_source;
  }

  // Field name: eph_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->eph_avail = tmp ? true : false;
  }

  // Field name: alm_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->alm_avail = tmp ? true : false;
  }

  // Field name: ano_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->ano_avail = tmp ? true : false;
  }

  // Field name: aop_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->aop_avail = tmp ? true : false;
  }

  // Field name: sbas_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->sbas_corr_used = tmp ? true : false;
  }

  // Field name: rtcm_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->rtcm_corr_used = tmp ? true : false;
  }

  // Field name: slas_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->slas_corr_used = tmp ? true : false;
  }

  // Field name: spartn_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->spartn_corr_used = tmp ? true : false;
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

  // Field name: clas_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->clas_corr_used = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t get_serialized_size_ublox_ubx_msgs__msg__SatFlags(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SatFlags__ros_msg_type * ros_message = static_cast<const _SatFlags__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name quality_ind
  {
    size_t item_size = sizeof(ros_message->quality_ind);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sv_used
  {
    size_t item_size = sizeof(ros_message->sv_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name health
  {
    size_t item_size = sizeof(ros_message->health);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name diff_corr
  {
    size_t item_size = sizeof(ros_message->diff_corr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name smoothed
  {
    size_t item_size = sizeof(ros_message->smoothed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name orbit_source
  {
    size_t item_size = sizeof(ros_message->orbit_source);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name eph_avail
  {
    size_t item_size = sizeof(ros_message->eph_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name alm_avail
  {
    size_t item_size = sizeof(ros_message->alm_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ano_avail
  {
    size_t item_size = sizeof(ros_message->ano_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name aop_avail
  {
    size_t item_size = sizeof(ros_message->aop_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sbas_corr_used
  {
    size_t item_size = sizeof(ros_message->sbas_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rtcm_corr_used
  {
    size_t item_size = sizeof(ros_message->rtcm_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name slas_corr_used
  {
    size_t item_size = sizeof(ros_message->slas_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name spartn_corr_used
  {
    size_t item_size = sizeof(ros_message->spartn_corr_used);
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
  // field.name clas_corr_used
  {
    size_t item_size = sizeof(ros_message->clas_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SatFlags__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_msgs__msg__SatFlags(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_msgs
size_t max_serialized_size_ublox_ubx_msgs__msg__SatFlags(
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

  // member: quality_ind
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sv_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: health
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
  // member: smoothed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: orbit_source
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: eph_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: alm_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ano_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: aop_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sbas_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rtcm_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: slas_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: spartn_corr_used
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
  // member: clas_corr_used
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
    using DataType = ublox_ubx_msgs__msg__SatFlags;
    is_plain =
      (
      offsetof(DataType, clas_corr_used) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SatFlags__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_msgs__msg__SatFlags(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SatFlags = {
  "ublox_ubx_msgs::msg",
  "SatFlags",
  _SatFlags__cdr_serialize,
  _SatFlags__cdr_deserialize,
  _SatFlags__get_serialized_size,
  _SatFlags__max_serialized_size
};

static rosidl_message_type_support_t _SatFlags__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SatFlags,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, SatFlags)() {
  return &_SatFlags__type_support;
}

#if defined(__cplusplus)
}
#endif
