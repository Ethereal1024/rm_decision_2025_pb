// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from iw_interfaces:msg/Robot.idl
// generated code does not contain a copyright notice

#ifndef IW_INTERFACES__MSG__DETAIL__ROBOT__FUNCTIONS_H_
#define IW_INTERFACES__MSG__DETAIL__ROBOT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "iw_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "iw_interfaces/msg/detail/robot__struct.h"

/// Initialize msg/Robot message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * iw_interfaces__msg__Robot
 * )) before or use
 * iw_interfaces__msg__Robot__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
bool
iw_interfaces__msg__Robot__init(iw_interfaces__msg__Robot * msg);

/// Finalize msg/Robot message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
void
iw_interfaces__msg__Robot__fini(iw_interfaces__msg__Robot * msg);

/// Create msg/Robot message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * iw_interfaces__msg__Robot__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
iw_interfaces__msg__Robot *
iw_interfaces__msg__Robot__create();

/// Destroy msg/Robot message.
/**
 * It calls
 * iw_interfaces__msg__Robot__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
void
iw_interfaces__msg__Robot__destroy(iw_interfaces__msg__Robot * msg);

/// Check for msg/Robot message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
bool
iw_interfaces__msg__Robot__are_equal(const iw_interfaces__msg__Robot * lhs, const iw_interfaces__msg__Robot * rhs);

/// Copy a msg/Robot message.
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
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
bool
iw_interfaces__msg__Robot__copy(
  const iw_interfaces__msg__Robot * input,
  iw_interfaces__msg__Robot * output);

/// Initialize array of msg/Robot messages.
/**
 * It allocates the memory for the number of elements and calls
 * iw_interfaces__msg__Robot__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
bool
iw_interfaces__msg__Robot__Sequence__init(iw_interfaces__msg__Robot__Sequence * array, size_t size);

/// Finalize array of msg/Robot messages.
/**
 * It calls
 * iw_interfaces__msg__Robot__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
void
iw_interfaces__msg__Robot__Sequence__fini(iw_interfaces__msg__Robot__Sequence * array);

/// Create array of msg/Robot messages.
/**
 * It allocates the memory for the array and calls
 * iw_interfaces__msg__Robot__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
iw_interfaces__msg__Robot__Sequence *
iw_interfaces__msg__Robot__Sequence__create(size_t size);

/// Destroy array of msg/Robot messages.
/**
 * It calls
 * iw_interfaces__msg__Robot__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
void
iw_interfaces__msg__Robot__Sequence__destroy(iw_interfaces__msg__Robot__Sequence * array);

/// Check for msg/Robot message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
bool
iw_interfaces__msg__Robot__Sequence__are_equal(const iw_interfaces__msg__Robot__Sequence * lhs, const iw_interfaces__msg__Robot__Sequence * rhs);

/// Copy an array of msg/Robot messages.
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
ROSIDL_GENERATOR_C_PUBLIC_iw_interfaces
bool
iw_interfaces__msg__Robot__Sequence__copy(
  const iw_interfaces__msg__Robot__Sequence * input,
  iw_interfaces__msg__Robot__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // IW_INTERFACES__MSG__DETAIL__ROBOT__FUNCTIONS_H_
