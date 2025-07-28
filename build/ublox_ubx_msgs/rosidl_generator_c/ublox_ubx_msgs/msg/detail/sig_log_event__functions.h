// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ublox_ubx_msgs:msg/SigLogEvent.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__FUNCTIONS_H_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ublox_ubx_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ublox_ubx_msgs/msg/detail/sig_log_event__struct.h"

/// Initialize msg/SigLogEvent message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ublox_ubx_msgs__msg__SigLogEvent
 * )) before or use
 * ublox_ubx_msgs__msg__SigLogEvent__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SigLogEvent__init(ublox_ubx_msgs__msg__SigLogEvent * msg);

/// Finalize msg/SigLogEvent message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
void
ublox_ubx_msgs__msg__SigLogEvent__fini(ublox_ubx_msgs__msg__SigLogEvent * msg);

/// Create msg/SigLogEvent message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ublox_ubx_msgs__msg__SigLogEvent__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
ublox_ubx_msgs__msg__SigLogEvent *
ublox_ubx_msgs__msg__SigLogEvent__create();

/// Destroy msg/SigLogEvent message.
/**
 * It calls
 * ublox_ubx_msgs__msg__SigLogEvent__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
void
ublox_ubx_msgs__msg__SigLogEvent__destroy(ublox_ubx_msgs__msg__SigLogEvent * msg);

/// Check for msg/SigLogEvent message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SigLogEvent__are_equal(const ublox_ubx_msgs__msg__SigLogEvent * lhs, const ublox_ubx_msgs__msg__SigLogEvent * rhs);

/// Copy a msg/SigLogEvent message.
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
ublox_ubx_msgs__msg__SigLogEvent__copy(
  const ublox_ubx_msgs__msg__SigLogEvent * input,
  ublox_ubx_msgs__msg__SigLogEvent * output);

/// Initialize array of msg/SigLogEvent messages.
/**
 * It allocates the memory for the number of elements and calls
 * ublox_ubx_msgs__msg__SigLogEvent__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SigLogEvent__Sequence__init(ublox_ubx_msgs__msg__SigLogEvent__Sequence * array, size_t size);

/// Finalize array of msg/SigLogEvent messages.
/**
 * It calls
 * ublox_ubx_msgs__msg__SigLogEvent__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
void
ublox_ubx_msgs__msg__SigLogEvent__Sequence__fini(ublox_ubx_msgs__msg__SigLogEvent__Sequence * array);

/// Create array of msg/SigLogEvent messages.
/**
 * It allocates the memory for the array and calls
 * ublox_ubx_msgs__msg__SigLogEvent__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
ublox_ubx_msgs__msg__SigLogEvent__Sequence *
ublox_ubx_msgs__msg__SigLogEvent__Sequence__create(size_t size);

/// Destroy array of msg/SigLogEvent messages.
/**
 * It calls
 * ublox_ubx_msgs__msg__SigLogEvent__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
void
ublox_ubx_msgs__msg__SigLogEvent__Sequence__destroy(ublox_ubx_msgs__msg__SigLogEvent__Sequence * array);

/// Check for msg/SigLogEvent message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ublox_ubx_msgs
bool
ublox_ubx_msgs__msg__SigLogEvent__Sequence__are_equal(const ublox_ubx_msgs__msg__SigLogEvent__Sequence * lhs, const ublox_ubx_msgs__msg__SigLogEvent__Sequence * rhs);

/// Copy an array of msg/SigLogEvent messages.
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
ublox_ubx_msgs__msg__SigLogEvent__Sequence__copy(
  const ublox_ubx_msgs__msg__SigLogEvent__Sequence * input,
  ublox_ubx_msgs__msg__SigLogEvent__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_LOG_EVENT__FUNCTIONS_H_
