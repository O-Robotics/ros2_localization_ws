// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ublox_ubx_msgs:msg/CarrSoln.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__CARR_SOLN__STRUCT_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__CARR_SOLN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CARRIER_SOLUTION_NO_CARRIER_RANGE_SOLUTION'.
enum
{
  ublox_ubx_msgs__msg__CarrSoln__CARRIER_SOLUTION_NO_CARRIER_RANGE_SOLUTION = 0
};

/// Constant 'CARRIER_SOLUTION_PHASE_WITH_FLOATING_AMBIGUITIES'.
enum
{
  ublox_ubx_msgs__msg__CarrSoln__CARRIER_SOLUTION_PHASE_WITH_FLOATING_AMBIGUITIES = 1
};

/// Constant 'CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES'.
enum
{
  ublox_ubx_msgs__msg__CarrSoln__CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES = 2
};

/// Struct defined in msg/CarrSoln in the package ublox_ubx_msgs.
typedef struct ublox_ubx_msgs__msg__CarrSoln
{
  /// Carrier phase solution status
  uint8_t status;
} ublox_ubx_msgs__msg__CarrSoln;

// Struct for a sequence of ublox_ubx_msgs__msg__CarrSoln.
typedef struct ublox_ubx_msgs__msg__CarrSoln__Sequence
{
  ublox_ubx_msgs__msg__CarrSoln * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ublox_ubx_msgs__msg__CarrSoln__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__CARR_SOLN__STRUCT_H_
