// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from task_manager_msgs:msg/TaskStatus.idl
// generated code does not contain a copyright notice

#ifndef TASK_MANAGER_MSGS__MSG__DETAIL__TASK_STATUS__FUNCTIONS_H_
#define TASK_MANAGER_MSGS__MSG__DETAIL__TASK_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "task_manager_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "task_manager_msgs/msg/detail/task_status__struct.h"

/// Initialize msg/TaskStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * task_manager_msgs__msg__TaskStatus
 * )) before or use
 * task_manager_msgs__msg__TaskStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
bool
task_manager_msgs__msg__TaskStatus__init(task_manager_msgs__msg__TaskStatus * msg);

/// Finalize msg/TaskStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
void
task_manager_msgs__msg__TaskStatus__fini(task_manager_msgs__msg__TaskStatus * msg);

/// Create msg/TaskStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * task_manager_msgs__msg__TaskStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
task_manager_msgs__msg__TaskStatus *
task_manager_msgs__msg__TaskStatus__create();

/// Destroy msg/TaskStatus message.
/**
 * It calls
 * task_manager_msgs__msg__TaskStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
void
task_manager_msgs__msg__TaskStatus__destroy(task_manager_msgs__msg__TaskStatus * msg);

/// Check for msg/TaskStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
bool
task_manager_msgs__msg__TaskStatus__are_equal(const task_manager_msgs__msg__TaskStatus * lhs, const task_manager_msgs__msg__TaskStatus * rhs);

/// Copy a msg/TaskStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
bool
task_manager_msgs__msg__TaskStatus__copy(
  const task_manager_msgs__msg__TaskStatus * input,
  task_manager_msgs__msg__TaskStatus * output);

/// Initialize array of msg/TaskStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * task_manager_msgs__msg__TaskStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
bool
task_manager_msgs__msg__TaskStatus__Sequence__init(task_manager_msgs__msg__TaskStatus__Sequence * array, size_t size);

/// Finalize array of msg/TaskStatus messages.
/**
 * It calls
 * task_manager_msgs__msg__TaskStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
void
task_manager_msgs__msg__TaskStatus__Sequence__fini(task_manager_msgs__msg__TaskStatus__Sequence * array);

/// Create array of msg/TaskStatus messages.
/**
 * It allocates the memory for the array and calls
 * task_manager_msgs__msg__TaskStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
task_manager_msgs__msg__TaskStatus__Sequence *
task_manager_msgs__msg__TaskStatus__Sequence__create(size_t size);

/// Destroy array of msg/TaskStatus messages.
/**
 * It calls
 * task_manager_msgs__msg__TaskStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
void
task_manager_msgs__msg__TaskStatus__Sequence__destroy(task_manager_msgs__msg__TaskStatus__Sequence * array);

/// Check for msg/TaskStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
bool
task_manager_msgs__msg__TaskStatus__Sequence__are_equal(const task_manager_msgs__msg__TaskStatus__Sequence * lhs, const task_manager_msgs__msg__TaskStatus__Sequence * rhs);

/// Copy an array of msg/TaskStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_task_manager_msgs
bool
task_manager_msgs__msg__TaskStatus__Sequence__copy(
  const task_manager_msgs__msg__TaskStatus__Sequence * input,
  task_manager_msgs__msg__TaskStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TASK_MANAGER_MSGS__MSG__DETAIL__TASK_STATUS__FUNCTIONS_H_
