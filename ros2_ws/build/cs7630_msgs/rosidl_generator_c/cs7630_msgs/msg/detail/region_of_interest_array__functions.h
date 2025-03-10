// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from cs7630_msgs:msg/RegionOfInterestArray.idl
// generated code does not contain a copyright notice

#ifndef CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__FUNCTIONS_H_
#define CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "cs7630_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "cs7630_msgs/msg/detail/region_of_interest_array__struct.h"

/// Initialize msg/RegionOfInterestArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cs7630_msgs__msg__RegionOfInterestArray
 * )) before or use
 * cs7630_msgs__msg__RegionOfInterestArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
bool
cs7630_msgs__msg__RegionOfInterestArray__init(cs7630_msgs__msg__RegionOfInterestArray * msg);

/// Finalize msg/RegionOfInterestArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
void
cs7630_msgs__msg__RegionOfInterestArray__fini(cs7630_msgs__msg__RegionOfInterestArray * msg);

/// Create msg/RegionOfInterestArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cs7630_msgs__msg__RegionOfInterestArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
cs7630_msgs__msg__RegionOfInterestArray *
cs7630_msgs__msg__RegionOfInterestArray__create();

/// Destroy msg/RegionOfInterestArray message.
/**
 * It calls
 * cs7630_msgs__msg__RegionOfInterestArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
void
cs7630_msgs__msg__RegionOfInterestArray__destroy(cs7630_msgs__msg__RegionOfInterestArray * msg);

/// Check for msg/RegionOfInterestArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
bool
cs7630_msgs__msg__RegionOfInterestArray__are_equal(const cs7630_msgs__msg__RegionOfInterestArray * lhs, const cs7630_msgs__msg__RegionOfInterestArray * rhs);

/// Copy a msg/RegionOfInterestArray message.
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
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
bool
cs7630_msgs__msg__RegionOfInterestArray__copy(
  const cs7630_msgs__msg__RegionOfInterestArray * input,
  cs7630_msgs__msg__RegionOfInterestArray * output);

/// Initialize array of msg/RegionOfInterestArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * cs7630_msgs__msg__RegionOfInterestArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
bool
cs7630_msgs__msg__RegionOfInterestArray__Sequence__init(cs7630_msgs__msg__RegionOfInterestArray__Sequence * array, size_t size);

/// Finalize array of msg/RegionOfInterestArray messages.
/**
 * It calls
 * cs7630_msgs__msg__RegionOfInterestArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
void
cs7630_msgs__msg__RegionOfInterestArray__Sequence__fini(cs7630_msgs__msg__RegionOfInterestArray__Sequence * array);

/// Create array of msg/RegionOfInterestArray messages.
/**
 * It allocates the memory for the array and calls
 * cs7630_msgs__msg__RegionOfInterestArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
cs7630_msgs__msg__RegionOfInterestArray__Sequence *
cs7630_msgs__msg__RegionOfInterestArray__Sequence__create(size_t size);

/// Destroy array of msg/RegionOfInterestArray messages.
/**
 * It calls
 * cs7630_msgs__msg__RegionOfInterestArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
void
cs7630_msgs__msg__RegionOfInterestArray__Sequence__destroy(cs7630_msgs__msg__RegionOfInterestArray__Sequence * array);

/// Check for msg/RegionOfInterestArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
bool
cs7630_msgs__msg__RegionOfInterestArray__Sequence__are_equal(const cs7630_msgs__msg__RegionOfInterestArray__Sequence * lhs, const cs7630_msgs__msg__RegionOfInterestArray__Sequence * rhs);

/// Copy an array of msg/RegionOfInterestArray messages.
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
ROSIDL_GENERATOR_C_PUBLIC_cs7630_msgs
bool
cs7630_msgs__msg__RegionOfInterestArray__Sequence__copy(
  const cs7630_msgs__msg__RegionOfInterestArray__Sequence * input,
  cs7630_msgs__msg__RegionOfInterestArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__FUNCTIONS_H_
