// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/MeasxData.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/measx_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/measx_data__struct.hpp"

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
  const ublox_ubx_msgs::msg::MeasxData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: gnss_id
  cdr << ros_message.gnss_id;
  // Member: sv_id
  cdr << ros_message.sv_id;
  // Member: c_no
  cdr << ros_message.c_no;
  // Member: mpath_indic
  cdr << ros_message.mpath_indic;
  // Member: doppler_ms
  cdr << ros_message.doppler_ms;
  // Member: doppler_hz
  cdr << ros_message.doppler_hz;
  // Member: whole_chips
  cdr << ros_message.whole_chips;
  // Member: frac_chips
  cdr << ros_message.frac_chips;
  // Member: code_phase
  cdr << ros_message.code_phase;
  // Member: int_code_phase
  cdr << ros_message.int_code_phase;
  // Member: pseu_range_rms_err
  cdr << ros_message.pseu_range_rms_err;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::MeasxData & ros_message)
{
  // Member: gnss_id
  cdr >> ros_message.gnss_id;

  // Member: sv_id
  cdr >> ros_message.sv_id;

  // Member: c_no
  cdr >> ros_message.c_no;

  // Member: mpath_indic
  cdr >> ros_message.mpath_indic;

  // Member: doppler_ms
  cdr >> ros_message.doppler_ms;

  // Member: doppler_hz
  cdr >> ros_message.doppler_hz;

  // Member: whole_chips
  cdr >> ros_message.whole_chips;

  // Member: frac_chips
  cdr >> ros_message.frac_chips;

  // Member: code_phase
  cdr >> ros_message.code_phase;

  // Member: int_code_phase
  cdr >> ros_message.int_code_phase;

  // Member: pseu_range_rms_err
  cdr >> ros_message.pseu_range_rms_err;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::MeasxData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: gnss_id
  {
    size_t item_size = sizeof(ros_message.gnss_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sv_id
  {
    size_t item_size = sizeof(ros_message.sv_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: c_no
  {
    size_t item_size = sizeof(ros_message.c_no);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mpath_indic
  {
    size_t item_size = sizeof(ros_message.mpath_indic);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: doppler_ms
  {
    size_t item_size = sizeof(ros_message.doppler_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: doppler_hz
  {
    size_t item_size = sizeof(ros_message.doppler_hz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: whole_chips
  {
    size_t item_size = sizeof(ros_message.whole_chips);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: frac_chips
  {
    size_t item_size = sizeof(ros_message.frac_chips);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: code_phase
  {
    size_t item_size = sizeof(ros_message.code_phase);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: int_code_phase
  {
    size_t item_size = sizeof(ros_message.int_code_phase);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pseu_range_rms_err
  {
    size_t item_size = sizeof(ros_message.pseu_range_rms_err);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_MeasxData(
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


  // Member: gnss_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sv_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: c_no
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: mpath_indic
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: doppler_ms
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: doppler_hz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: whole_chips
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: frac_chips
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: code_phase
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: int_code_phase
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pseu_range_rms_err
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
    using DataType = ublox_ubx_msgs::msg::MeasxData;
    is_plain =
      (
      offsetof(DataType, pseu_range_rms_err) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _MeasxData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::MeasxData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MeasxData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::MeasxData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MeasxData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::MeasxData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MeasxData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MeasxData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MeasxData__callbacks = {
  "ublox_ubx_msgs::msg",
  "MeasxData",
  _MeasxData__cdr_serialize,
  _MeasxData__cdr_deserialize,
  _MeasxData__get_serialized_size,
  _MeasxData__max_serialized_size
};

static rosidl_message_type_support_t _MeasxData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MeasxData__callbacks,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::MeasxData>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_MeasxData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, MeasxData)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_MeasxData__handle;
}

#ifdef __cplusplus
}
#endif
