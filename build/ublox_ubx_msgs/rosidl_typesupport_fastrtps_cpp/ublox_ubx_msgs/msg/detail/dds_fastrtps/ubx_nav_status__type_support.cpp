// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavStatus.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__struct.hpp"

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
bool cdr_serialize(
  const ublox_ubx_msgs::msg::GpsFix &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ublox_ubx_msgs::msg::GpsFix &);
size_t get_serialized_size(
  const ublox_ubx_msgs::msg::GpsFix &,
  size_t current_alignment);
size_t
max_serialized_size_GpsFix(
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
bool cdr_serialize(
  const ublox_ubx_msgs::msg::MapMatching &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ublox_ubx_msgs::msg::MapMatching &);
size_t get_serialized_size(
  const ublox_ubx_msgs::msg::MapMatching &,
  size_t current_alignment);
size_t
max_serialized_size_MapMatching(
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
bool cdr_serialize(
  const ublox_ubx_msgs::msg::PSMStatus &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ublox_ubx_msgs::msg::PSMStatus &);
size_t get_serialized_size(
  const ublox_ubx_msgs::msg::PSMStatus &,
  size_t current_alignment);
size_t
max_serialized_size_PSMStatus(
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
bool cdr_serialize(
  const ublox_ubx_msgs::msg::SpoofDet &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ublox_ubx_msgs::msg::SpoofDet &);
size_t get_serialized_size(
  const ublox_ubx_msgs::msg::SpoofDet &,
  size_t current_alignment);
size_t
max_serialized_size_SpoofDet(
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
bool cdr_serialize(
  const ublox_ubx_msgs::msg::CarrSoln &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ublox_ubx_msgs::msg::CarrSoln &);
size_t get_serialized_size(
  const ublox_ubx_msgs::msg::CarrSoln &,
  size_t current_alignment);
size_t
max_serialized_size_CarrSoln(
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
  const ublox_ubx_msgs::msg::UBXNavStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: itow
  cdr << ros_message.itow;
  // Member: gps_fix
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.gps_fix,
    cdr);
  // Member: gps_fix_ok
  cdr << (ros_message.gps_fix_ok ? true : false);
  // Member: diff_soln
  cdr << (ros_message.diff_soln ? true : false);
  // Member: wkn_set
  cdr << (ros_message.wkn_set ? true : false);
  // Member: tow_set
  cdr << (ros_message.tow_set ? true : false);
  // Member: diff_corr
  cdr << (ros_message.diff_corr ? true : false);
  // Member: carr_soln_valid
  cdr << (ros_message.carr_soln_valid ? true : false);
  // Member: map_matching
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.map_matching,
    cdr);
  // Member: psm
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.psm,
    cdr);
  // Member: spoof_det
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.spoof_det,
    cdr);
  // Member: carr_soln
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.carr_soln,
    cdr);
  // Member: ttff
  cdr << ros_message.ttff;
  // Member: msss
  cdr << ros_message.msss;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::UBXNavStatus & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: itow
  cdr >> ros_message.itow;

  // Member: gps_fix
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.gps_fix);

  // Member: gps_fix_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.gps_fix_ok = tmp ? true : false;
  }

  // Member: diff_soln
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.diff_soln = tmp ? true : false;
  }

  // Member: wkn_set
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.wkn_set = tmp ? true : false;
  }

  // Member: tow_set
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.tow_set = tmp ? true : false;
  }

  // Member: diff_corr
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.diff_corr = tmp ? true : false;
  }

  // Member: carr_soln_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.carr_soln_valid = tmp ? true : false;
  }

  // Member: map_matching
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.map_matching);

  // Member: psm
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.psm);

  // Member: spoof_det
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.spoof_det);

  // Member: carr_soln
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.carr_soln);

  // Member: ttff
  cdr >> ros_message.ttff;

  // Member: msss
  cdr >> ros_message.msss;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::UBXNavStatus & ros_message,
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
  // Member: itow
  {
    size_t item_size = sizeof(ros_message.itow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gps_fix

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.gps_fix, current_alignment);
  // Member: gps_fix_ok
  {
    size_t item_size = sizeof(ros_message.gps_fix_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: diff_soln
  {
    size_t item_size = sizeof(ros_message.diff_soln);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wkn_set
  {
    size_t item_size = sizeof(ros_message.wkn_set);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tow_set
  {
    size_t item_size = sizeof(ros_message.tow_set);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: diff_corr
  {
    size_t item_size = sizeof(ros_message.diff_corr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: carr_soln_valid
  {
    size_t item_size = sizeof(ros_message.carr_soln_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: map_matching

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.map_matching, current_alignment);
  // Member: psm

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.psm, current_alignment);
  // Member: spoof_det

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.spoof_det, current_alignment);
  // Member: carr_soln

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.carr_soln, current_alignment);
  // Member: ttff
  {
    size_t item_size = sizeof(ros_message.ttff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: msss
  {
    size_t item_size = sizeof(ros_message.msss);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_UBXNavStatus(
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

  // Member: itow
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: gps_fix
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_GpsFix(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: gps_fix_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: diff_soln
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: wkn_set
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: tow_set
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

  // Member: carr_soln_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: map_matching
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_MapMatching(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: psm
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PSMStatus(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: spoof_det
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_SpoofDet(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: carr_soln
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_CarrSoln(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: ttff
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: msss
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
    using DataType = ublox_ubx_msgs::msg::UBXNavStatus;
    is_plain =
      (
      offsetof(DataType, msss) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _UBXNavStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::UBXNavStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _UBXNavStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::UBXNavStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _UBXNavStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::UBXNavStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _UBXNavStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_UBXNavStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _UBXNavStatus__callbacks = {
  "ublox_ubx_msgs::msg",
  "UBXNavStatus",
  _UBXNavStatus__cdr_serialize,
  _UBXNavStatus__cdr_deserialize,
  _UBXNavStatus__get_serialized_size,
  _UBXNavStatus__max_serialized_size
};

static rosidl_message_type_support_t _UBXNavStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_UBXNavStatus__callbacks,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::UBXNavStatus>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_UBXNavStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, UBXNavStatus)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_UBXNavStatus__handle;
}

#ifdef __cplusplus
}
#endif
