// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/SigFlags.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/sig_flags__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/sig_flags__struct.hpp"

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
  const ublox_ubx_msgs::msg::SigFlags & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: health
  cdr << ros_message.health;
  // Member: pr_smoothed
  cdr << (ros_message.pr_smoothed ? true : false);
  // Member: pr_used
  cdr << (ros_message.pr_used ? true : false);
  // Member: cr_used
  cdr << (ros_message.cr_used ? true : false);
  // Member: do_used
  cdr << (ros_message.do_used ? true : false);
  // Member: pr_corr_used
  cdr << (ros_message.pr_corr_used ? true : false);
  // Member: cr_corr_used
  cdr << (ros_message.cr_corr_used ? true : false);
  // Member: do_corr_used
  cdr << (ros_message.do_corr_used ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::SigFlags & ros_message)
{
  // Member: health
  cdr >> ros_message.health;

  // Member: pr_smoothed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.pr_smoothed = tmp ? true : false;
  }

  // Member: pr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.pr_used = tmp ? true : false;
  }

  // Member: cr_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.cr_used = tmp ? true : false;
  }

  // Member: do_used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.do_used = tmp ? true : false;
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

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::SigFlags & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: health
  {
    size_t item_size = sizeof(ros_message.health);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pr_smoothed
  {
    size_t item_size = sizeof(ros_message.pr_smoothed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pr_used
  {
    size_t item_size = sizeof(ros_message.pr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cr_used
  {
    size_t item_size = sizeof(ros_message.cr_used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: do_used
  {
    size_t item_size = sizeof(ros_message.do_used);
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

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_SigFlags(
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


  // Member: health
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pr_smoothed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cr_used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: do_used
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ublox_ubx_msgs::msg::SigFlags;
    is_plain =
      (
      offsetof(DataType, do_corr_used) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SigFlags__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::SigFlags *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SigFlags__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::SigFlags *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SigFlags__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::SigFlags *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SigFlags__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SigFlags(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SigFlags__callbacks = {
  "ublox_ubx_msgs::msg",
  "SigFlags",
  _SigFlags__cdr_serialize,
  _SigFlags__cdr_deserialize,
  _SigFlags__get_serialized_size,
  _SigFlags__max_serialized_size
};

static rosidl_message_type_support_t _SigFlags__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SigFlags__callbacks,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::SigFlags>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_SigFlags__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, SigFlags)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_SigFlags__handle;
}

#ifdef __cplusplus
}
#endif
