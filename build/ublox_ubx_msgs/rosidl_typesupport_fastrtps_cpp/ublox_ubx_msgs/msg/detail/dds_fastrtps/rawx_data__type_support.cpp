// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/RawxData.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/rawx_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/rawx_data__struct.hpp"

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
  const ublox_ubx_msgs::msg::TrkStat &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ublox_ubx_msgs::msg::TrkStat &);
size_t get_serialized_size(
  const ublox_ubx_msgs::msg::TrkStat &,
  size_t current_alignment);
size_t
max_serialized_size_TrkStat(
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
  const ublox_ubx_msgs::msg::RawxData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pr_mes
  cdr << ros_message.pr_mes;
  // Member: cp_mes
  cdr << ros_message.cp_mes;
  // Member: do_mes
  cdr << ros_message.do_mes;
  // Member: gnss_id
  cdr << ros_message.gnss_id;
  // Member: sv_id
  cdr << ros_message.sv_id;
  // Member: sig_id
  cdr << ros_message.sig_id;
  // Member: freq_id
  cdr << ros_message.freq_id;
  // Member: locktime
  cdr << ros_message.locktime;
  // Member: c_no
  cdr << ros_message.c_no;
  // Member: pr_stdev
  cdr << ros_message.pr_stdev;
  // Member: cp_stdev
  cdr << ros_message.cp_stdev;
  // Member: do_stdev
  cdr << ros_message.do_stdev;
  // Member: trk_stat
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.trk_stat,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::RawxData & ros_message)
{
  // Member: pr_mes
  cdr >> ros_message.pr_mes;

  // Member: cp_mes
  cdr >> ros_message.cp_mes;

  // Member: do_mes
  cdr >> ros_message.do_mes;

  // Member: gnss_id
  cdr >> ros_message.gnss_id;

  // Member: sv_id
  cdr >> ros_message.sv_id;

  // Member: sig_id
  cdr >> ros_message.sig_id;

  // Member: freq_id
  cdr >> ros_message.freq_id;

  // Member: locktime
  cdr >> ros_message.locktime;

  // Member: c_no
  cdr >> ros_message.c_no;

  // Member: pr_stdev
  cdr >> ros_message.pr_stdev;

  // Member: cp_stdev
  cdr >> ros_message.cp_stdev;

  // Member: do_stdev
  cdr >> ros_message.do_stdev;

  // Member: trk_stat
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.trk_stat);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::RawxData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pr_mes
  {
    size_t item_size = sizeof(ros_message.pr_mes);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cp_mes
  {
    size_t item_size = sizeof(ros_message.cp_mes);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: do_mes
  {
    size_t item_size = sizeof(ros_message.do_mes);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
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
  // Member: locktime
  {
    size_t item_size = sizeof(ros_message.locktime);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: c_no
  {
    size_t item_size = sizeof(ros_message.c_no);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pr_stdev
  {
    size_t item_size = sizeof(ros_message.pr_stdev);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cp_stdev
  {
    size_t item_size = sizeof(ros_message.cp_stdev);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: do_stdev
  {
    size_t item_size = sizeof(ros_message.do_stdev);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: trk_stat

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.trk_stat, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_RawxData(
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


  // Member: pr_mes
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: cp_mes
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: do_mes
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

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

  // Member: locktime
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: c_no
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pr_stdev
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cp_stdev
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: do_stdev
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: trk_stat
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_TrkStat(
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
    using DataType = ublox_ubx_msgs::msg::RawxData;
    is_plain =
      (
      offsetof(DataType, trk_stat) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RawxData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::RawxData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RawxData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::RawxData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RawxData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::RawxData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RawxData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RawxData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RawxData__callbacks = {
  "ublox_ubx_msgs::msg",
  "RawxData",
  _RawxData__cdr_serialize,
  _RawxData__cdr_deserialize,
  _RawxData__get_serialized_size,
  _RawxData__max_serialized_size
};

static rosidl_message_type_support_t _RawxData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RawxData__callbacks,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::RawxData>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_RawxData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, RawxData)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_RawxData__handle;
}

#ifdef __cplusplus
}
#endif
