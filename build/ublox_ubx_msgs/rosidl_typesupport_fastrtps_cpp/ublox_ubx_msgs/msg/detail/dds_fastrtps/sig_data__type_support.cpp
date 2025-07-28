// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/SigData.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sig_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/sig_data__struct.hpp"

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
bool cdr_serialize(
  const ublox_ubx_msgs::msg::SigFlags &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ublox_ubx_msgs::msg::SigFlags &);
size_t get_serialized_size(
  const ublox_ubx_msgs::msg::SigFlags &,
  size_t current_alignment);
size_t
max_serialized_size_SigFlags(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ublox_ubx_msgs


namespace ublox_ubx_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_serialize(
  const ublox_ubx_msgs::msg::SigData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: gnss_id
  cdr << ros_message.gnss_id;
  // Member: sv_id
  cdr << ros_message.sv_id;
  // Member: sig_id
  cdr << ros_message.sig_id;
  // Member: freq_id
  cdr << ros_message.freq_id;
  // Member: pr_res
  cdr << ros_message.pr_res;
  // Member: cno
  cdr << ros_message.cno;
  // Member: quality_ind
  cdr << ros_message.quality_ind;
  // Member: corr_source
  cdr << ros_message.corr_source;
  // Member: iono_model
  cdr << ros_message.iono_model;
  // Member: sig_flags
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.sig_flags,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::SigData & ros_message)
{
  // Member: gnss_id
  cdr >> ros_message.gnss_id;

  // Member: sv_id
  cdr >> ros_message.sv_id;

  // Member: sig_id
  cdr >> ros_message.sig_id;

  // Member: freq_id
  cdr >> ros_message.freq_id;

  // Member: pr_res
  cdr >> ros_message.pr_res;

  // Member: cno
  cdr >> ros_message.cno;

  // Member: quality_ind
  cdr >> ros_message.quality_ind;

  // Member: corr_source
  cdr >> ros_message.corr_source;

  // Member: iono_model
  cdr >> ros_message.iono_model;

  // Member: sig_flags
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.sig_flags);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::SigData & ros_message,
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
  // Member: sig_id
  {
    size_t item_size = sizeof(ros_message.sig_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: freq_id
  {
    size_t item_size = sizeof(ros_message.freq_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pr_res
  {
    size_t item_size = sizeof(ros_message.pr_res);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cno
  {
    size_t item_size = sizeof(ros_message.cno);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: quality_ind
  {
    size_t item_size = sizeof(ros_message.quality_ind);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: corr_source
  {
    size_t item_size = sizeof(ros_message.corr_source);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: iono_model
  {
    size_t item_size = sizeof(ros_message.iono_model);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sig_flags

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.sig_flags, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_SigData(
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

  // Member: sig_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: freq_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pr_res
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: cno
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: quality_ind
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: corr_source
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: iono_model
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sig_flags
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_SigFlags(
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
    using DataType = ublox_ubx_msgs::msg::SigData;
    is_plain =
      (
      offsetof(DataType, sig_flags) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SigData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::SigData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SigData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::SigData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SigData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::SigData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SigData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SigData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SigData__callbacks = {
  "ublox_ubx_msgs::msg",
  "SigData",
  _SigData__cdr_serialize,
  _SigData__cdr_deserialize,
  _SigData__get_serialized_size,
  _SigData__max_serialized_size
};

static rosidl_message_type_support_t _SigData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SigData__callbacks,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::SigData>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_SigData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, SigData)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_SigData__handle;
}

#ifdef __cplusplus
}
#endif
