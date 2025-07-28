// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/SatFlags.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sat_flags__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/sat_flags__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace ublox_ubx_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_serialize(
  const ublox_ubx_msgs::msg::SatFlags & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: quality_ind
  cdr << ros_message.quality_ind;
  // Member: sv_used
  cdr << (ros_message.sv_used ? true : false);
  // Member: health
  cdr << ros_message.health;
  // Member: diff_corr
  cdr << (ros_message.diff_corr ? true : false);
  // Member: smoothed
  cdr << (ros_message.smoothed ? true : false);
  // Member: orbit_source
  cdr << ros_message.orbit_source;
  // Member: eph_avail
  cdr << (ros_message.eph_avail ? true : false);
  // Member: alm_avail
  cdr << (ros_message.alm_avail ? true : false);
  // Member: ano_avail
  cdr << (ros_message.ano_avail ? true : false);
  // Member: aop_avail
  cdr << (ros_message.aop_avail ? true : false);
  // Member: sbas_corr_used
  cdr << (ros_message.sbas_corr_used ? true : false);
  // Member: rtcm_corr_used
  cdr << (ros_message.rtcm_corr_used ? true : false);
  // Member: slas_corr_used
  cdr << (ros_message.slas_corr_used ? true : false);
  // Member: spartn_corr_used
  cdr << (ros_message.spartn_corr_used ? true : false);
  // Member: pr_corr_used
  cdr << (ros_message.pr_corr_used ? true : false);
  // Member: cr_corr_used
  cdr << (ros_message.cr_corr_used ? true : false);
  // Member: do_corr_used
  cdr << (ros_message.do_corr_used ? true : false);
  // Member: clas_corr_used
  cdr << (ros_message.clas_corr_used ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::SatFlags & ros_message)
{
  // Member: quality_ind
  cdr >> ros_message.quality_ind;

  // Member: sv_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.sv_used = tmp ? true : false;
  }

  // Member: health
  cdr >> ros_message.health;

  // Member: diff_corr
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.diff_corr = tmp ? true : false;
  }

  // Member: smoothed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.smoothed = tmp ? true : false;
  }

  // Member: orbit_source
  cdr >> ros_message.orbit_source;

  // Member: eph_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.eph_avail = tmp ? true : false;
  }

  // Member: alm_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.alm_avail = tmp ? true : false;
  }

  // Member: ano_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ano_avail = tmp ? true : false;
  }

  // Member: aop_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.aop_avail = tmp ? true : false;
  }

  // Member: sbas_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.sbas_corr_used = tmp ? true : false;
  }

  // Member: rtcm_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.rtcm_corr_used = tmp ? true : false;
  }

  // Member: slas_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.slas_corr_used = tmp ? true : false;
  }

  // Member: spartn_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.spartn_corr_used = tmp ? true : false;
  }

  // Member: pr_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.pr_corr_used = tmp ? true : false;
  }

  // Member: cr_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.cr_corr_used = tmp ? true : false;
  }

  // Member: do_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.do_corr_used = tmp ? true : false;
  }

  // Member: clas_corr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.clas_corr_used = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::SatFlags & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: quality_ind
  {
    size_t item_size = sizeof(ros_message.quality_ind);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sv_used
  {
    size_t item_size = sizeof(ros_message.sv_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: health
  {
    size_t item_size = sizeof(ros_message.health);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: diff_corr
  {
    size_t item_size = sizeof(ros_message.diff_corr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: smoothed
  {
    size_t item_size = sizeof(ros_message.smoothed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: orbit_source
  {
    size_t item_size = sizeof(ros_message.orbit_source);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: eph_avail
  {
    size_t item_size = sizeof(ros_message.eph_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: alm_avail
  {
    size_t item_size = sizeof(ros_message.alm_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ano_avail
  {
    size_t item_size = sizeof(ros_message.ano_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: aop_avail
  {
    size_t item_size = sizeof(ros_message.aop_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sbas_corr_used
  {
    size_t item_size = sizeof(ros_message.sbas_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rtcm_corr_used
  {
    size_t item_size = sizeof(ros_message.rtcm_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: slas_corr_used
  {
    size_t item_size = sizeof(ros_message.slas_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: spartn_corr_used
  {
    size_t item_size = sizeof(ros_message.spartn_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pr_corr_used
  {
    size_t item_size = sizeof(ros_message.pr_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cr_corr_used
  {
    size_t item_size = sizeof(ros_message.cr_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: do_corr_used
  {
    size_t item_size = sizeof(ros_message.do_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: clas_corr_used
  {
    size_t item_size = sizeof(ros_message.clas_corr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_SatFlags(
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


  // Member: quality_ind
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sv_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: health
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: diff_corr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: smoothed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: orbit_source
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: eph_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: alm_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ano_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: aop_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sbas_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rtcm_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: slas_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: spartn_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pr_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cr_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: do_corr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: clas_corr_used
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
    using DataType = ublox_ubx_msgs::msg::SatFlags;
    is_plain =
      (
      offsetof(DataType, clas_corr_used) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SatFlags__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::SatFlags *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SatFlags__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::SatFlags *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SatFlags__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::SatFlags *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SatFlags__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SatFlags(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SatFlags__callbacks = {
  "ublox_ubx_msgs::msg",
  "SatFlags",
  _SatFlags__cdr_serialize,
  _SatFlags__cdr_deserialize,
  _SatFlags__get_serialized_size,
  _SatFlags__max_serialized_size
};

static rosidl_message_type_support_t _SatFlags__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SatFlags__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ublox_ubx_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ublox_ubx_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<ublox_ubx_msgs::msg::SatFlags>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_SatFlags__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, SatFlags)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_SatFlags__handle;
}

#ifdef __cplusplus
}
#endif
