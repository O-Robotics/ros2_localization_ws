// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_interfaces:srv/WarmStart.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_INTERFACES__SRV__DETAIL__WARM_START__STRUCT_H_
#define UBLOX_UBX_INTERFACES__SRV__DETAIL__WARM_START__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/WarmStart in the package ublox_ubx_interfaces.
typedef struct ublox_ubx_interfaces__srv__WarmStart_Request
{
  uint8_t reset_type;
} ublox_ubx_interfaces__srv__WarmStart_Request;

// Struct for a sequence of ublox_ubx_interfaces__srv__WarmStart_Request.
typedef struct ublox_ubx_interfaces__srv__WarmStart_Request__Sequence
{
  ublox_ubx_interfaces__srv__WarmStart_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_interfaces__srv__WarmStart_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/WarmStart in the package ublox_ubx_interfaces.
typedef struct ublox_ubx_interfaces__srv__WarmStart_Response
{
  uint8_t structure_needs_at_least_one_member;
} ublox_ubx_interfaces__srv__WarmStart_Response;

// Struct for a sequence of ublox_ubx_interfaces__srv__WarmStart_Response.
typedef struct ublox_ubx_interfaces__srv__WarmStart_Response__Sequence
{
  ublox_ubx_interfaces__srv__WarmStart_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_interfaces__srv__WarmStart_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_INTERFACES__SRV__DETAIL__WARM_START__STRUCT_H_
