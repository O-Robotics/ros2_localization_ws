// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_interfaces:srv/ResetODO.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_INTERFACES__SRV__DETAIL__RESET_ODO__STRUCT_H_
#define UBLOX_UBX_INTERFACES__SRV__DETAIL__RESET_ODO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ResetODO in the package ublox_ubx_interfaces.
typedef struct ublox_ubx_interfaces__srv__ResetODO_Request
{
  uint8_t structure_needs_at_least_one_member;
} ublox_ubx_interfaces__srv__ResetODO_Request;

// Struct for a sequence of ublox_ubx_interfaces__srv__ResetODO_Request.
typedef struct ublox_ubx_interfaces__srv__ResetODO_Request__Sequence
{
  ublox_ubx_interfaces__srv__ResetODO_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_interfaces__srv__ResetODO_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ResetODO in the package ublox_ubx_interfaces.
typedef struct ublox_ubx_interfaces__srv__ResetODO_Response
{
  uint8_t structure_needs_at_least_one_member;
} ublox_ubx_interfaces__srv__ResetODO_Response;

// Struct for a sequence of ublox_ubx_interfaces__srv__ResetODO_Response.
typedef struct ublox_ubx_interfaces__srv__ResetODO_Response__Sequence
{
  ublox_ubx_interfaces__srv__ResetODO_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_interfaces__srv__ResetODO_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_INTERFACES__SRV__DETAIL__RESET_ODO__STRUCT_H_
