// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from task_manager_msgs:msg/SyncStatus.idl
// generated code does not contain a copyright notice
#include "task_manager_msgs/msg/detail/sync_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
task_manager_msgs__msg__SyncStatus__init(task_manager_msgs__msg__SyncStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    task_manager_msgs__msg__SyncStatus__fini(msg);
    return false;
  }
  // status
  return true;
}

void
task_manager_msgs__msg__SyncStatus__fini(task_manager_msgs__msg__SyncStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // status
}

bool
task_manager_msgs__msg__SyncStatus__are_equal(const task_manager_msgs__msg__SyncStatus * lhs, const task_manager_msgs__msg__SyncStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
task_manager_msgs__msg__SyncStatus__copy(
  const task_manager_msgs__msg__SyncStatus * input,
  task_manager_msgs__msg__SyncStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // status
  output->status = input->status;
  return true;
}

task_manager_msgs__msg__SyncStatus *
task_manager_msgs__msg__SyncStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager_msgs__msg__SyncStatus * msg = (task_manager_msgs__msg__SyncStatus *)allocator.allocate(sizeof(task_manager_msgs__msg__SyncStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager_msgs__msg__SyncStatus));
  bool success = task_manager_msgs__msg__SyncStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager_msgs__msg__SyncStatus__destroy(task_manager_msgs__msg__SyncStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager_msgs__msg__SyncStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager_msgs__msg__SyncStatus__Sequence__init(task_manager_msgs__msg__SyncStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager_msgs__msg__SyncStatus * data = NULL;

  if (size) {
    data = (task_manager_msgs__msg__SyncStatus *)allocator.zero_allocate(size, sizeof(task_manager_msgs__msg__SyncStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager_msgs__msg__SyncStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager_msgs__msg__SyncStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager_msgs__msg__SyncStatus__Sequence__fini(task_manager_msgs__msg__SyncStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager_msgs__msg__SyncStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager_msgs__msg__SyncStatus__Sequence *
task_manager_msgs__msg__SyncStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager_msgs__msg__SyncStatus__Sequence * array = (task_manager_msgs__msg__SyncStatus__Sequence *)allocator.allocate(sizeof(task_manager_msgs__msg__SyncStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager_msgs__msg__SyncStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager_msgs__msg__SyncStatus__Sequence__destroy(task_manager_msgs__msg__SyncStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager_msgs__msg__SyncStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager_msgs__msg__SyncStatus__Sequence__are_equal(const task_manager_msgs__msg__SyncStatus__Sequence * lhs, const task_manager_msgs__msg__SyncStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager_msgs__msg__SyncStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager_msgs__msg__SyncStatus__Sequence__copy(
  const task_manager_msgs__msg__SyncStatus__Sequence * input,
  task_manager_msgs__msg__SyncStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager_msgs__msg__SyncStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager_msgs__msg__SyncStatus * data =
      (task_manager_msgs__msg__SyncStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager_msgs__msg__SyncStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager_msgs__msg__SyncStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager_msgs__msg__SyncStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
