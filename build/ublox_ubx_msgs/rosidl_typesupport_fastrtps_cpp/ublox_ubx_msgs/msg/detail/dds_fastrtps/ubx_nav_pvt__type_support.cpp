// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPVT.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__struct.hpp"

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
  const ublox_ubx_msgs::msg::PSMPVT &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ublox_ubx_msgs::msg::PSMPVT &);
size_t get_serialized_size(
  const ublox_ubx_msgs::msg::PSMPVT &,
  size_t current_alignment);
size_t
max_serialized_size_PSMPVT(
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
  const ublox_ubx_msgs::msg::UBXNavPVT & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: itow
  cdr << ros_message.itow;
  // Member: year
  cdr << ros_message.year;
  // Member: month
  cdr << ros_message.month;
  // Member: day
  cdr << ros_message.day;
  // Member: hour
  cdr << ros_message.hour;
  // Member: min
  cdr << ros_message.min;
  // Member: sec
  cdr << ros_message.sec;
  // Member: valid_date
  cdr << (ros_message.valid_date ? true : false);
  // Member: valid_time
  cdr << (ros_message.valid_time ? true : false);
  // Member: fully_resolved
  cdr << (ros_message.fully_resolved ? true : false);
  // Member: valid_mag
  cdr << (ros_message.valid_mag ? true : false);
  // Member: t_acc
  cdr << ros_message.t_acc;
  // Member: nano
  cdr << ros_message.nano;
  // Member: gps_fix
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.gps_fix,
    cdr);
  // Member: gnss_fix_ok
  cdr << (ros_message.gnss_fix_ok ? true : false);
  // Member: diff_soln
  cdr << (ros_message.diff_soln ? true : false);
  // Member: psm
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.psm,
    cdr);
  // Member: head_veh_valid
  cdr << (ros_message.head_veh_valid ? true : false);
  // Member: carr_soln
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.carr_soln,
    cdr);
  // Member: confirmed_avail
  cdr << (ros_message.confirmed_avail ? true : false);
  // Member: confirmed_date
  cdr << (ros_message.confirmed_date ? true : false);
  // Member: confirmed_time
  cdr << (ros_message.confirmed_time ? true : false);
  // Member: num_sv
  cdr << ros_message.num_sv;
  // Member: lon
  cdr << ros_message.lon;
  // Member: lat
  cdr << ros_message.lat;
  // Member: height
  cdr << ros_message.height;
  // Member: hmsl
  cdr << ros_message.hmsl;
  // Member: h_acc
  cdr << ros_message.h_acc;
  // Member: v_acc
  cdr << ros_message.v_acc;
  // Member: vel_n
  cdr << ros_message.vel_n;
  // Member: vel_e
  cdr << ros_message.vel_e;
  // Member: vel_d
  cdr << ros_message.vel_d;
  // Member: g_speed
  cdr << ros_message.g_speed;
  // Member: head_mot
  cdr << ros_message.head_mot;
  // Member: s_acc
  cdr << ros_message.s_acc;
  // Member: head_acc
  cdr << ros_message.head_acc;
  // Member: p_dop
  cdr << ros_message.p_dop;
  // Member: invalid_llh
  cdr << (ros_message.invalid_llh ? true : false);
  // Member: head_veh
  cdr << ros_message.head_veh;
  // Member: mag_dec
  cdr << ros_message.mag_dec;
  // Member: mag_acc
  cdr << ros_message.mag_acc;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::UBXNavPVT & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: itow
  cdr >> ros_message.itow;

  // Member: year
  cdr >> ros_message.year;

  // Member: month
  cdr >> ros_message.month;

  // Member: day
  cdr >> ros_message.day;

  // Member: hour
  cdr >> ros_message.hour;

  // Member: min
  cdr >> ros_message.min;

  // Member: sec
  cdr >> ros_message.sec;

  // Member: valid_date
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.valid_date = tmp ? true : false;
  }

  // Member: valid_time
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.valid_time = tmp ? true : false;
  }

  // Member: fully_resolved
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fully_resolved = tmp ? true : false;
  }

  // Member: valid_mag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.valid_mag = tmp ? true : false;
  }

  // Member: t_acc
  cdr >> ros_message.t_acc;

  // Member: nano
  cdr >> ros_message.nano;

  // Member: gps_fix
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.gps_fix);

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

  // Member: psm
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.psm);

  // Member: head_veh_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.head_veh_valid = tmp ? true : false;
  }

  // Member: carr_soln
  ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.carr_soln);

  // Member: confirmed_avail
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.confirmed_avail = tmp ? true : false;
  }

  // Member: confirmed_date
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.confirmed_date = tmp ? true : false;
  }

  // Member: confirmed_time
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.confirmed_time = tmp ? true : false;
  }

  // Member: num_sv
  cdr >> ros_message.num_sv;

  // Member: lon
  cdr >> ros_message.lon;

  // Member: lat
  cdr >> ros_message.lat;

  // Member: height
  cdr >> ros_message.height;

  // Member: hmsl
  cdr >> ros_message.hmsl;

  // Member: h_acc
  cdr >> ros_message.h_acc;

  // Member: v_acc
  cdr >> ros_message.v_acc;

  // Member: vel_n
  cdr >> ros_message.vel_n;

  // Member: vel_e
  cdr >> ros_message.vel_e;

  // Member: vel_d
  cdr >> ros_message.vel_d;

  // Member: g_speed
  cdr >> ros_message.g_speed;

  // Member: head_mot
  cdr >> ros_message.head_mot;

  // Member: s_acc
  cdr >> ros_message.s_acc;

  // Member: head_acc
  cdr >> ros_message.head_acc;

  // Member: p_dop
  cdr >> ros_message.p_dop;

  // Member: invalid_llh
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.invalid_llh = tmp ? true : false;
  }

  // Member: head_veh
  cdr >> ros_message.head_veh;

  // Member: mag_dec
  cdr >> ros_message.mag_dec;

  // Member: mag_acc
  cdr >> ros_message.mag_acc;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::UBXNavPVT & ros_message,
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
  // Member: year
  {
    size_t item_size = sizeof(ros_message.year);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: month
  {
    size_t item_size = sizeof(ros_message.month);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: day
  {
    size_t item_size = sizeof(ros_message.day);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: hour
  {
    size_t item_size = sizeof(ros_message.hour);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: min
  {
    size_t item_size = sizeof(ros_message.min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sec
  {
    size_t item_size = sizeof(ros_message.sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: valid_date
  {
    size_t item_size = sizeof(ros_message.valid_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: valid_time
  {
    size_t item_size = sizeof(ros_message.valid_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fully_resolved
  {
    size_t item_size = sizeof(ros_message.fully_resolved);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: valid_mag
  {
    size_t item_size = sizeof(ros_message.valid_mag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: t_acc
  {
    size_t item_size = sizeof(ros_message.t_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: nano
  {
    size_t item_size = sizeof(ros_message.nano);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gps_fix

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.gps_fix, current_alignment);
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
  // Member: psm

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.psm, current_alignment);
  // Member: head_veh_valid
  {
    size_t item_size = sizeof(ros_message.head_veh_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: carr_soln

  current_alignment +=
    ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.carr_soln, current_alignment);
  // Member: confirmed_avail
  {
    size_t item_size = sizeof(ros_message.confirmed_avail);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: confirmed_date
  {
    size_t item_size = sizeof(ros_message.confirmed_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: confirmed_time
  {
    size_t item_size = sizeof(ros_message.confirmed_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: num_sv
  {
    size_t item_size = sizeof(ros_message.num_sv);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: lon
  {
    size_t item_size = sizeof(ros_message.lon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: lat
  {
    size_t item_size = sizeof(ros_message.lat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: height
  {
    size_t item_size = sizeof(ros_message.height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: hmsl
  {
    size_t item_size = sizeof(ros_message.hmsl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: h_acc
  {
    size_t item_size = sizeof(ros_message.h_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: v_acc
  {
    size_t item_size = sizeof(ros_message.v_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vel_n
  {
    size_t item_size = sizeof(ros_message.vel_n);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vel_e
  {
    size_t item_size = sizeof(ros_message.vel_e);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vel_d
  {
    size_t item_size = sizeof(ros_message.vel_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: g_speed
  {
    size_t item_size = sizeof(ros_message.g_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: head_mot
  {
    size_t item_size = sizeof(ros_message.head_mot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: s_acc
  {
    size_t item_size = sizeof(ros_message.s_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: head_acc
  {
    size_t item_size = sizeof(ros_message.head_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: p_dop
  {
    size_t item_size = sizeof(ros_message.p_dop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: invalid_llh
  {
    size_t item_size = sizeof(ros_message.invalid_llh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: head_veh
  {
    size_t item_size = sizeof(ros_message.head_veh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mag_dec
  {
    size_t item_size = sizeof(ros_message.mag_dec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mag_acc
  {
    size_t item_size = sizeof(ros_message.mag_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_UBXNavPVT(
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

  // Member: year
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: month
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: day
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: hour
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: valid_date
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: valid_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fully_resolved
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: valid_mag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: t_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: nano
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

  // Member: psm
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PSMPVT(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: head_veh_valid
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

  // Member: confirmed_avail
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: confirmed_date
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: confirmed_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: num_sv
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: lon
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: lat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: height
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: hmsl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: h_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: v_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel_n
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel_e
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: g_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: head_mot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: s_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: head_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: p_dop
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: invalid_llh
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: head_veh
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: mag_dec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: mag_acc
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
    using DataType = ublox_ubx_msgs::msg::UBXNavPVT;
    is_plain =
      (
      offsetof(DataType, mag_acc) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _UBXNavPVT__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::UBXNavPVT *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _UBXNavPVT__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::UBXNavPVT *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _UBXNavPVT__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::UBXNavPVT *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _UBXNavPVT__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_UBXNavPVT(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _UBXNavPVT__callbacks = {
  "ublox_ubx_msgs::msg",
  "UBXNavPVT",
  _UBXNavPVT__cdr_serialize,
  _UBXNavPVT__cdr_deserialize,
  _UBXNavPVT__get_serialized_size,
  _UBXNavPVT__max_serialized_size
};

static rosidl_message_type_support_t _UBXNavPVT__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_UBXNavPVT__callbacks,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::UBXNavPVT>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_UBXNavPVT__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, UBXNavPVT)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_UBXNavPVT__handle;
}

#ifdef __cplusplus
}
#endif
