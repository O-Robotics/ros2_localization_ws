// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavRelPosNED.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__struct.hpp"

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
  const ublox_ubx_msgs::msg::UBXNavRelPosNED & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: version
  cdr << ros_message.version;
  // Member: ref_station_id
  cdr << ros_message.ref_station_id;
  // Member: itow
  cdr << ros_message.itow;
  // Member: rel_pos_n
  cdr << ros_message.rel_pos_n;
  // Member: rel_pos_e
  cdr << ros_message.rel_pos_e;
  // Member: rel_pos_d
  cdr << ros_message.rel_pos_d;
  // Member: rel_pos_length
  cdr << ros_message.rel_pos_length;
  // Member: rel_pos_heading
  cdr << ros_message.rel_pos_heading;
  // Member: rel_pos_hp_n
  cdr << ros_message.rel_pos_hp_n;
  // Member: rel_pos_hp_e
  cdr << ros_message.rel_pos_hp_e;
  // Member: rel_pos_hp_d
  cdr << ros_message.rel_pos_hp_d;
  // Member: rel_pos_hp_length
  cdr << ros_message.rel_pos_hp_length;
  // Member: acc_n
  cdr << ros_message.acc_n;
  // Member: acc_e
  cdr << ros_message.acc_e;
  // Member: acc_d
  cdr << ros_message.acc_d;
  // Member: acc_length
  cdr << ros_message.acc_length;
  // Member: acc_heading
  cdr << ros_message.acc_heading;
  // Member: gnss_fix_ok
  cdr << (ros_message.gnss_fix_ok ? true : false);
  // Member: diff_soln
  cdr << (ros_message.diff_soln ? true : false);
  // Member: rel_pos_valid
  cdr << (ros_message.rel_pos_valid ? true : false);
  // Member: carr_soln
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.carr_soln,
    cdr);
  // Member: is_moving
  cdr << (ros_message.is_moving ? true : false);
  // Member: ref_pos_miss
  cdr << (ros_message.ref_pos_miss ? true : false);
  // Member: ref_obs_miss
  cdr << (ros_message.ref_obs_miss ? true : false);
  // Member: rel_pos_heading_valid
  cdr << (ros_message.rel_pos_heading_valid ? true : false);
  // Member: rel_pos_normalized
  cdr << (ros_message.rel_pos_normalized ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::UBXNavRelPosNED & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: version
  cdr >> ros_message.version;

  // Member: ref_station_id
  cdr >> ros_message.ref_station_id;

  // Member: itow
  cdr >> ros_message.itow;

  // Member: rel_pos_n
  cdr >> ros_message.rel_pos_n;

  // Member: rel_pos_e
  cdr >> ros_message.rel_pos_e;

  // Member: rel_pos_d
  cdr >> ros_message.rel_pos_d;

  // Member: rel_pos_length
  cdr >> ros_message.rel_pos_length;

  // Member: rel_pos_heading
  cdr >> ros_message.rel_pos_heading;

  // Member: rel_pos_hp_n
  cdr >> ros_message.rel_pos_hp_n;

  // Member: rel_pos_hp_e
  cdr >> ros_message.rel_pos_hp_e;

  // Member: rel_pos_hp_d
  cdr >> ros_message.rel_pos_hp_d;

  // Member: rel_pos_hp_length
  cdr >> ros_message.rel_pos_hp_length;

  // Member: acc_n
  cdr >> ros_message.acc_n;

  // Member: acc_e
  cdr >> ros_message.acc_e;

  // Member: acc_d
  cdr >> ros_message.acc_d;

  // Member: acc_length
  cdr >> ros_message.acc_length;

  // Member: acc_heading
  cdr >> ros_message.acc_heading;

  // Member: gnss_fix_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.gnss_fix_ok = tmp ? true : false;
  }

  // Member: diff_soln
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.diff_soln = tmp ? true : false;
  }

  // Member: rel_pos_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.rel_pos_valid = tmp ? true : false;
  }

  // Member: carr_soln
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.carr_soln);

  // Member: is_moving
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_moving = tmp ? true : false;
  }

  // Member: ref_pos_miss
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ref_pos_miss = tmp ? true : false;
  }

  // Member: ref_obs_miss
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ref_obs_miss = tmp ? true : false;
  }

  // Member: rel_pos_heading_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.rel_pos_heading_valid = tmp ? true : false;
  }

  // Member: rel_pos_normalized
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.rel_pos_normalized = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::UBXNavRelPosNED & ros_message,
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
  // Member: ref_station_id
  {
    size_t item_size = sizeof(ros_message.ref_station_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: itow
  {
    size_t item_size = sizeof(ros_message.itow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_n
  {
    size_t item_size = sizeof(ros_message.rel_pos_n);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_e
  {
    size_t item_size = sizeof(ros_message.rel_pos_e);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_d
  {
    size_t item_size = sizeof(ros_message.rel_pos_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_length
  {
    size_t item_size = sizeof(ros_message.rel_pos_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_heading
  {
    size_t item_size = sizeof(ros_message.rel_pos_heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_hp_n
  {
    size_t item_size = sizeof(ros_message.rel_pos_hp_n);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_hp_e
  {
    size_t item_size = sizeof(ros_message.rel_pos_hp_e);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_hp_d
  {
    size_t item_size = sizeof(ros_message.rel_pos_hp_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_hp_length
  {
    size_t item_size = sizeof(ros_message.rel_pos_hp_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acc_n
  {
    size_t item_size = sizeof(ros_message.acc_n);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acc_e
  {
    size_t item_size = sizeof(ros_message.acc_e);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acc_d
  {
    size_t item_size = sizeof(ros_message.acc_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acc_length
  {
    size_t item_size = sizeof(ros_message.acc_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acc_heading
  {
    size_t item_size = sizeof(ros_message.acc_heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gnss_fix_ok
  {
    size_t item_size = sizeof(ros_message.gnss_fix_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: diff_soln
  {
    size_t item_size = sizeof(ros_message.diff_soln);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_valid
  {
    size_t item_size = sizeof(ros_message.rel_pos_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: carr_soln

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.carr_soln, current_alignment);
  // Member: is_moving
  {
    size_t item_size = sizeof(ros_message.is_moving);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ref_pos_miss
  {
    size_t item_size = sizeof(ros_message.ref_pos_miss);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ref_obs_miss
  {
    size_t item_size = sizeof(ros_message.ref_obs_miss);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_heading_valid
  {
    size_t item_size = sizeof(ros_message.rel_pos_heading_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rel_pos_normalized
  {
    size_t item_size = sizeof(ros_message.rel_pos_normalized);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_UBXNavRelPosNED(
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

  // Member: ref_station_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: itow
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rel_pos_n
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rel_pos_e
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rel_pos_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rel_pos_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rel_pos_heading
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rel_pos_hp_n
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rel_pos_hp_e
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rel_pos_hp_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rel_pos_hp_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: acc_n
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: acc_e
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: acc_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: acc_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: acc_heading
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: gnss_fix_ok
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

  // Member: rel_pos_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
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

  // Member: is_moving
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ref_pos_miss
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ref_obs_miss
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rel_pos_heading_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rel_pos_normalized
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
    using DataType = ublox_ubx_msgs::msg::UBXNavRelPosNED;
    is_plain =
      (
      offsetof(DataType, rel_pos_normalized) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _UBXNavRelPosNED__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::UBXNavRelPosNED *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _UBXNavRelPosNED__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::UBXNavRelPosNED *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _UBXNavRelPosNED__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::UBXNavRelPosNED *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _UBXNavRelPosNED__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_UBXNavRelPosNED(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _UBXNavRelPosNED__callbacks = {
  "ublox_ubx_msgs::msg",
  "UBXNavRelPosNED",
  _UBXNavRelPosNED__cdr_serialize,
  _UBXNavRelPosNED__cdr_deserialize,
  _UBXNavRelPosNED__get_serialized_size,
  _UBXNavRelPosNED__max_serialized_size
};

static rosidl_message_type_support_t _UBXNavRelPosNED__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_UBXNavRelPosNED__callbacks,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::UBXNavRelPosNED>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_UBXNavRelPosNED__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, UBXNavRelPosNED)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_UBXNavRelPosNED__handle;
}

#ifdef __cplusplus
}
#endif
