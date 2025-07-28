// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ublox_ubx_msgs:msg/SatInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__FUNCTIONS_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ublox_ubx_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ublox_ubx_msgs/msg/detail/sat_info__struct.h"

/// Initialize msg/SatInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ublox_ubx_msgs__msg__SatInfo
 * )) before or use
 * ublox_ubx_msgs__msg__SatInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SatInfo__init(ublox_ubx_msgs__msg__SatInfo * msg);

/// Finalize msg/SatInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
void
ublox_ubx_msgs__msg__SatInfo__fini(ublox_ubx_msgs__msg__SatInfo * msg);

/// Create msg/SatInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ublox_ubx_msgs__msg__SatInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
ublox_ubx_msgs__msg__SatInfo *
ublox_ubx_msgs__msg__SatInfo__create();

/// Destroy msg/SatInfo message.
/**
 * It calls
 * ublox_ubx_msgs__msg__SatInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
void
ublox_ubx_msgs__msg__SatInfo__destroy(ublox_ubx_msgs__msg__SatInfo * msg);

/// Check for msg/SatInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SatInfo__are_equal(const ublox_ubx_msgs__msg__SatInfo * lhs, const ublox_ubx_msgs__msg__SatInfo * rhs);

/// Copy a msg/SatInfo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SatInfo__copy(
  const ublox_ubx_msgs__msg__SatInfo * input,
  ublox_ubx_msgs__msg__SatInfo * output);

/// Initialize array of msg/SatInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * ublox_ubx_msgs__msg__SatInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SatInfo__Sequence__init(ublox_ubx_msgs__msg__SatInfo__Sequence * array, size_t size);

/// Finalize array of msg/SatInfo messages.
/**
 * It calls
 * ublox_ubx_msgs__msg__SatInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
void
ublox_ubx_msgs__msg__SatInfo__Sequence__fini(ublox_ubx_msgs__msg__SatInfo__Sequence * array);

/// Create array of msg/SatInfo messages.
/**
 * It allocates the memory for the array and calls
 * ublox_ubx_msgs__msg__SatInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
ublox_ubx_msgs__msg__SatInfo__Sequence *
ublox_ubx_msgs__msg__SatInfo__Sequence__create(size_t size);

/// Destroy array of msg/SatInfo messages.
/**
 * It calls
 * ublox_ubx_msgs__msg__SatInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
void
ublox_ubx_msgs__msg__SatInfo__Sequence__destroy(ublox_ubx_msgs__msg__SatInfo__Sequence * array);

/// Check for msg/SatInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SatInfo__Sequence__are_equal(const ublox_ubx_msgs__msg__SatInfo__Sequence * lhs, const ublox_ubx_msgs__msg__SatInfo__Sequence * rhs);

/// Copy an array of msg/SatInfo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SatInfo__Sequence__copy(
  const ublox_ubx_msgs__msg__SatInfo__Sequence * input,
  ublox_ubx_msgs__msg__SatInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__FUNCTIONS_H_
