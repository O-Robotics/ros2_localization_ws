// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ublox_ubx_interfaces:srv/ColdStart.idl
// generated code does not contain a copyright notice
#include "ublox_ubx_interfaces/srv/detail/cold_start__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ublox_ubx_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_interfaces/srv/detail/cold_start__struct.h"
#include "ublox_ubx_interfaces/srv/detail/cold_start__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ColdStart_Request__ros_msg_type = ublox_ubx_interfaces__srv__ColdStart_Request;

static bool _ColdStart_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ColdStart_Request__ros_msg_type * ros_message = static_cast<const _ColdStart_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: reset_type
  {
    cdr << ros_message->reset_type;
  }

  return true;
}

static bool _ColdStart_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ColdStart_Request__ros_msg_type * ros_message = static_cast<_ColdStart_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: reset_type
  {
    cdr >> ros_message->reset_type;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_interfaces
size_t get_serialized_size_ublox_ubx_interfaces__srv__ColdStart_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ColdStart_Request__ros_msg_type * ros_message = static_cast<const _ColdStart_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name reset_type
  {
    size_t item_size = sizeof(ros_message->reset_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ColdStart_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_interfaces__srv__ColdStart_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_interfaces
size_t max_serialized_size_ublox_ubx_interfaces__srv__ColdStart_Request(
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

  // member: reset_type
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
    using DataType = ublox_ubx_interfaces__srv__ColdStart_Request;
    is_plain =
      (
      offsetof(DataType, reset_type) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ColdStart_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_interfaces__srv__ColdStart_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ColdStart_Request = {
  "ublox_ubx_interfaces::srv",
  "ColdStart_Request",
  _ColdStart_Request__cdr_serialize,
  _ColdStart_Request__cdr_deserialize,
  _ColdStart_Request__get_serialized_size,
  _ColdStart_Request__max_serialized_size
};

static rosidl_message_type_support_t _ColdStart_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ColdStart_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_interfaces, srv, ColdStart_Request)() {
  return &_ColdStart_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ublox_ubx_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ublox_ubx_interfaces/srv/detail/cold_start__struct.h"
// already included above
// #include "ublox_ubx_interfaces/srv/detail/cold_start__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ColdStart_Response__ros_msg_type = ublox_ubx_interfaces__srv__ColdStart_Response;

static bool _ColdStart_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ColdStart_Response__ros_msg_type * ros_message = static_cast<const _ColdStart_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _ColdStart_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ColdStart_Response__ros_msg_type * ros_message = static_cast<_ColdStart_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_interfaces
size_t get_serialized_size_ublox_ubx_interfaces__srv__ColdStart_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ColdStart_Response__ros_msg_type * ros_message = static_cast<const _ColdStart_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ColdStart_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ublox_ubx_interfaces__srv__ColdStart_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ublox_ubx_interfaces
size_t max_serialized_size_ublox_ubx_interfaces__srv__ColdStart_Response(
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

  // member: structure_needs_at_least_one_member
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
    using DataType = ublox_ubx_interfaces__srv__ColdStart_Response;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ColdStart_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ublox_ubx_interfaces__srv__ColdStart_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ColdStart_Response = {
  "ublox_ubx_interfaces::srv",
  "ColdStart_Response",
  _ColdStart_Response__cdr_serialize,
  _ColdStart_Response__cdr_deserialize,
  _ColdStart_Response__get_serialized_size,
  _ColdStart_Response__max_serialized_size
};

static rosidl_message_type_support_t _ColdStart_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ColdStart_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_interfaces, srv, ColdStart_Response)() {
  return &_ColdStart_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "ublox_ubx_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ublox_ubx_interfaces/srv/cold_start.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t ColdStart__callbacks = {
  "ublox_ubx_interfaces::srv",
  "ColdStart",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_interfaces, srv, ColdStart_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_interfaces, srv, ColdStart_Response)(),
};

static rosidl_service_type_support_t ColdStart__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &ColdStart__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_interfaces, srv, ColdStart)() {
  return &ColdStart__handle;
}

#if defined(__cplusplus)
}
#endif
