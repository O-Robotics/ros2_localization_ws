// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_interfaces:srv/ColdStart.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_INTERFACES__SRV__DETAIL__COLD_START__STRUCT_H_
#define UBLOX_UBX_INTERFACES__SRV__DETAIL__COLD_START__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'HW_RESET_IMMEDIATELY'.
enum
{
  ublox_ubx_interfaces__srv__ColdStart_Request__HW_RESET_IMMEDIATELY = 0
};

/// Constant 'SW_RESET_CONTROLLED'.
enum
{
  ublox_ubx_interfaces__srv__ColdStart_Request__SW_RESET_CONTROLLED = 1
};

/// Constant 'SW_RESET_CONTROLLED_GNSS'.
enum
{
  ublox_ubx_interfaces__srv__ColdStart_Request__SW_RESET_CONTROLLED_GNSS = 2
};

/// Constant 'HW_RESET_AFTER_SHUTDOWN'.
enum
{
  ublox_ubx_interfaces__srv__ColdStart_Request__HW_RESET_AFTER_SHUTDOWN = 4
};

/// Constant 'GNSS_STOP_CONTROLLED'.
enum
{
  ublox_ubx_interfaces__srv__ColdStart_Request__GNSS_STOP_CONTROLLED = 8
};

/// Constant 'GNSS_START_CONTROLLED'.
enum
{
  ublox_ubx_interfaces__srv__ColdStart_Request__GNSS_START_CONTROLLED = 9
};

/// Struct defined in srv/ColdStart in the package ublox_ubx_interfaces.
typedef struct ublox_ubx_interfaces__srv__ColdStart_Request
{
  uint8_t reset_type;
} ublox_ubx_interfaces__srv__ColdStart_Request;

// Struct for a sequence of ublox_ubx_interfaces__srv__ColdStart_Request.
typedef struct ublox_ubx_interfaces__srv__ColdStart_Request__Sequence
{
  ublox_ubx_interfaces__srv__ColdStart_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_interfaces__srv__ColdStart_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ColdStart in the package ublox_ubx_interfaces.
typedef struct ublox_ubx_interfaces__srv__ColdStart_Response
{
  uint8_t structure_needs_at_least_one_member;
} ublox_ubx_interfaces__srv__ColdStart_Response;

// Struct for a sequence of ublox_ubx_interfaces__srv__ColdStart_Response.
typedef struct ublox_ubx_interfaces__srv__ColdStart_Response__Sequence
{
  ublox_ubx_interfaces__srv__ColdStart_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_interfaces__srv__ColdStart_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_INTERFACES__SRV__DETAIL__COLD_START__STRUCT_H_
