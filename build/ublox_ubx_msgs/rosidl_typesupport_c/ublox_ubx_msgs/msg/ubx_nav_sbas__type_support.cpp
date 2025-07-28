// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from ublox_ubx_msgs:msg/UBXNavSBAS.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_sbas__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_sbas__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ublox_ubx_msgs
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _UBXNavSBAS_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _UBXNavSBAS_type_support_ids_t;

static const _UBXNavSBAS_type_support_ids_t _UBXNavSBAS_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _UBXNavSBAS_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _UBXNavSBAS_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _UBXNavSBAS_type_support_symbol_names_t _UBXNavSBAS_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ublox_ubx_msgs, msg, UBXNavSBAS)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_ubx_msgs, msg, UBXNavSBAS)),
  }
};

typedef struct _UBXNavSBAS_type_support_data_t
{
  void * data[2];
} _UBXNavSBAS_type_support_data_t;

static _UBXNavSBAS_type_support_data_t _UBXNavSBAS_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _UBXNavSBAS_message_typesupport_map = {
  2,
  "ublox_ubx_msgs",
  &_UBXNavSBAS_message_typesupport_ids.typesupport_identifier[0],
  &_UBXNavSBAS_message_typesupport_symbol_names.symbol_name[0],
  &_UBXNavSBAS_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t UBXNavSBAS_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_UBXNavSBAS_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace ublox_ubx_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ublox_ubx_msgs, msg, UBXNavSBAS)() {
  return &::ublox_ubx_msgs::msg::rosidl_typesupport_c::UBXNavSBAS_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
