// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/ESFSensorStatus.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__struct.hpp"

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
  const ublox_ubx_msgs::msg::ESFSensorStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: sensor_data_type
  cdr << ros_message.sensor_data_type;
  // Member: used
  cdr << (ros_message.used ? true : false);
  // Member: ready
  cdr << (ros_message.ready ? true : false);
  // Member: calib_status
  cdr << ros_message.calib_status;
  // Member: time_status
  cdr << ros_message.time_status;
  // Member: freq
  cdr << ros_message.freq;
  // Member: fault_bad_meas
  cdr << (ros_message.fault_bad_meas ? true : false);
  // Member: fault_bad_ttag
  cdr << (ros_message.fault_bad_ttag ? true : false);
  // Member: fault_missing_meas
  cdr << (ros_message.fault_missing_meas ? true : false);
  // Member: fault_noisy_meas
  cdr << (ros_message.fault_noisy_meas ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ublox_ubx_msgs::msg::ESFSensorStatus & ros_message)
{
  // Member: sensor_data_type
  cdr >> ros_message.sensor_data_type;

  // Member: used
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.used = tmp ? true : false;
  }

  // Member: ready
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ready = tmp ? true : false;
  }

  // Member: calib_status
  cdr >> ros_message.calib_status;

  // Member: time_status
  cdr >> ros_message.time_status;

  // Member: freq
  cdr >> ros_message.freq;

  // Member: fault_bad_meas
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fault_bad_meas = tmp ? true : false;
  }

  // Member: fault_bad_ttag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fault_bad_ttag = tmp ? true : false;
  }

  // Member: fault_missing_meas
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fault_missing_meas = tmp ? true : false;
  }

  // Member: fault_noisy_meas
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fault_noisy_meas = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
get_serialized_size(
  const ublox_ubx_msgs::msg::ESFSensorStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: sensor_data_type
  {
    size_t item_size = sizeof(ros_message.sensor_data_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: used
  {
    size_t item_size = sizeof(ros_message.used);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ready
  {
    size_t item_size = sizeof(ros_message.ready);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: calib_status
  {
    size_t item_size = sizeof(ros_message.calib_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: time_status
  {
    size_t item_size = sizeof(ros_message.time_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: freq
  {
    size_t item_size = sizeof(ros_message.freq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fault_bad_meas
  {
    size_t item_size = sizeof(ros_message.fault_bad_meas);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fault_bad_ttag
  {
    size_t item_size = sizeof(ros_message.fault_bad_ttag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fault_missing_meas
  {
    size_t item_size = sizeof(ros_message.fault_missing_meas);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fault_noisy_meas
  {
    size_t item_size = sizeof(ros_message.fault_noisy_meas);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ublox_ubx_msgs
max_serialized_size_ESFSensorStatus(
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


  // Member: sensor_data_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: used
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ready
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: calib_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: time_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: freq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fault_bad_meas
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fault_bad_ttag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fault_missing_meas
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fault_noisy_meas
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
    using DataType = ublox_ubx_msgs::msg::ESFSensorStatus;
    is_plain =
      (
      offsetof(DataType, fault_noisy_meas) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ESFSensorStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::ESFSensorStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ESFSensorStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ublox_ubx_msgs::msg::ESFSensorStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ESFSensorStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ublox_ubx_msgs::msg::ESFSensorStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ESFSensorStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ESFSensorStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ESFSensorStatus__callbacks = {
  "ublox_ubx_msgs::msg",
  "ESFSensorStatus",
  _ESFSensorStatus__cdr_serialize,
  _ESFSensorStatus__cdr_deserialize,
  _ESFSensorStatus__get_serialized_size,
  _ESFSensorStatus__max_serialized_size
};

static rosidl_message_type_support_t _ESFSensorStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ESFSensorStatus__callbacks,
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
get_message_type_support_handle<ublox_ubx_msgs::msg::ESFSensorStatus>()
{
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_ESFSensorStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ublox_ubx_msgs, msg, ESFSensorStatus)() {
  return &ublox_ubx_msgs::msg::typesupport_fastrtps_cpp::_ESFSensorStatus__handle;
}

#ifdef __cplusplus
}
#endif
