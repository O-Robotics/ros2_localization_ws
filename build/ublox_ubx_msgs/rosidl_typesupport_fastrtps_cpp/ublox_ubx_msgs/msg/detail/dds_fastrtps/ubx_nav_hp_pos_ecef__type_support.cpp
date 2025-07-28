// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosECEF.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace ublox_ubx_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_serialize(
  const ublox_ubx_msgs::msg::UBXNavHPPosECEF & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: version
  cdr << ros_message.version;
  // Member: itow
  cdr << ros_message.itow;
  // Member: ecef_x
  cdr << ros_message.ecef_x;
  // Member: ecef_y
  cdr << ros_message.ecef_y;
  // Member: ecef_z
  cdr << ros_message.ecef_z;
  // Member: ecef_x_hp
  cdr << ros_message.ecef_x_hp;
  // Member: ecef_y_hp
  cdr << ros_message.ecef_y_hp;
  // Member: ecef_z_hp
  cdr << ros_message.ecef_z_hp;
  // Member: invalid_ecef_x
  cdr << (ros_message.invalid_ecef_x ? true : false);
  // Member: invalid_ecef_y
  cdr << (ros_message.invalid_ecef_y ? true : false);
  // Member: invalid_ecef_z
  cdr << (ros_message.invalid_ecef_z ? true : false);
  // Member: invalid_ecef_x_hp
  cdr << (ros_message.invalid_ecef_x_hp ? true : false);
  // Member: invalid_ecef_y_hp
  cdr << (ros_message.invalid_ecef_y_hp ? true : false);
  // Member: invalid_ecef_z_hp
  cdr << (ros_message.invalid_ecef_z_hp ? true : false);
  // Member: p_acc
  cdr << ros_message.p_acc;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::UBXNavHPPosECEF & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: version
  cdr >> ros_message.version;

  // Member: itow
  cdr >> ros_message.itow;

  // Member: ecef_x
  cdr >> ros_message.ecef_x;

  // Member: ecef_y
  cdr >> ros_message.ecef_y;

  // Member: ecef_z
  cdr >> ros_message.ecef_z;

  // Member: ecef_x_hp
  cdr >> ros_message.ecef_x_hp;

  // Member: ecef_y_hp
  cdr >> ros_message.ecef_y_hp;

  // Member: ecef_z_hp
  cdr >> ros_message.ecef_z_hp;

  // Member: invalid_ecef_x
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.invalid_ecef_x = tmp ? true : false;
  }

  // Member: invalid_ecef_y
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.invalid_ecef_y = tmp ? true : false;
  }

  // Member: invalid_ecef_z
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.invalid_ecef_z = tmp ? true : false;
  }

  // Member: invalid_ecef_x_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.invalid_ecef_x_hp = tmp ? true : false;
  }

  // Member: invalid_ecef_y_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.invalid_ecef_y_hp = tmp ? true : false;
  }

  // Member: invalid_ecef_z_hp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.invalid_ecef_z_hp = tmp ? true : false;
  }

  // Member: p_acc
  cdr >> ros_message.p_acc;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::UBXNavHPPosECEF & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: version
  {
    size_t item_size = sizeof(ros_message.version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: itow
  {
    size_t item_size = sizeof(ros_message.itow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ecef_x
  {
    size_t item_size = sizeof(ros_message.ecef_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ecef_y
  {
    size_t item_size = sizeof(ros_message.ecef_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ecef_z
  {
    size_t item_size = sizeof(ros_message.ecef_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ecef_x_hp
  {
    size_t item_size = sizeof(ros_message.ecef_x_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ecef_y_hp
  {
    size_t item_size = sizeof(ros_message.ecef_y_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ecef_z_hp
  {
    size_t item_size = sizeof(ros_message.ecef_z_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: invalid_ecef_x
  {
    size_t item_size = sizeof(ros_message.invalid_ecef_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: invalid_ecef_y
  {
    size_t item_size = sizeof(ros_message.invalid_ecef_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: invalid_ecef_z
  {
    size_t item_size = sizeof(ros_message.invalid_ecef_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: invalid_ecef_x_hp
  {
    size_t item_size = sizeof(ros_message.invalid_ecef_x_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: invalid_ecef_y_hp
  {
    size_t item_size = sizeof(ros_message.invalid_ecef_y_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: invalid_ecef_z_hp
  {
    size_t item_size = sizeof(ros_message.invalid_ecef_z_hp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: p_acc
  {
    size_t item_size = sizeof(ros_message.p_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_UBXNavHPPosECEF(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: version
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: itow
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ecef_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ecef_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ecef_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ecef_x_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ecef_y_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ecef_z_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: invalid_ecef_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: invalid_ecef_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: invalid_ecef_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: invalid_ecef_x_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: invalid_ecef_y_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: invalid_ecef_z_hp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: p_acc
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
    using DataType = ublox_ubx_msgs::msg::UBXNavHPPosECEF;
    is_plain =
      (
      offsetof(DataType, p_acc) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _UBXNavHPPosECEF__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::UBXNavHPPosECEF *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _UBXNavHPPosECEF__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::UBXNavHPPosECEF *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _UBXNavHPPosECEF__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::UBXNavHPPosECEF *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _UBXNavHPPosECEF__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_UBXNavHPPosECEF(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _UBXNavHPPosECEF__callbacks = {
  "ublox_ubx_msgs::msg",
  "UBXNavHPPosECEF",
  _UBXNavHPPosECEF__cdr_serialize,
  _UBXNavHPPosECEF__cdr_deserialize,
  _UBXNavHPPosECEF__get_serialized_size,
  _UBXNavHPPosECEF__max_serialized_size
};

static rosidl_message_type_support_t _UBXNavHPPosECEF__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_UBXNavHPPosECEF__callbacks,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::UBXNavHPPosECEF>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_UBXNavHPPosECEF__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, UBXNavHPPosECEF)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_UBXNavHPPosECEF__handle;
}

#ifdef __cplusplus
}
#endif
