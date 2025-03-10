// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cs7630_msgs:msg/RegionOfInterestArray.idl
// generated code does not contain a copyright notice
#include "cs7630_msgs/msg/detail/region_of_interest_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `regions`
#include "sensor_msgs/msg/detail/region_of_interest__functions.h"

bool
cs7630_msgs__msg__RegionOfInterestArray__init(cs7630_msgs__msg__RegionOfInterestArray * msg)
{
  if (!msg) {
    return false;
  }
  // regions
  if (!sensor_msgs__msg__RegionOfInterest__Sequence__init(&msg->regions, 0)) {
    cs7630_msgs__msg__RegionOfInterestArray__fini(msg);
    return false;
  }
  return true;
}

void
cs7630_msgs__msg__RegionOfInterestArray__fini(cs7630_msgs__msg__RegionOfInterestArray * msg)
{
  if (!msg) {
    return;
  }
  // regions
  sensor_msgs__msg__RegionOfInterest__Sequence__fini(&msg->regions);
}

bool
cs7630_msgs__msg__RegionOfInterestArray__are_equal(const cs7630_msgs__msg__RegionOfInterestArray * lhs, const cs7630_msgs__msg__RegionOfInterestArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // regions
  if (!sensor_msgs__msg__RegionOfInterest__Sequence__are_equal(
      &(lhs->regions), &(rhs->regions)))
  {
    return false;
  }
  return true;
}

bool
cs7630_msgs__msg__RegionOfInterestArray__copy(
  const cs7630_msgs__msg__RegionOfInterestArray * input,
  cs7630_msgs__msg__RegionOfInterestArray * output)
{
  if (!input || !output) {
    return false;
  }
  // regions
  if (!sensor_msgs__msg__RegionOfInterest__Sequence__copy(
      &(input->regions), &(output->regions)))
  {
    return false;
  }
  return true;
}

cs7630_msgs__msg__RegionOfInterestArray *
cs7630_msgs__msg__RegionOfInterestArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cs7630_msgs__msg__RegionOfInterestArray * msg = (cs7630_msgs__msg__RegionOfInterestArray *)allocator.allocate(sizeof(cs7630_msgs__msg__RegionOfInterestArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cs7630_msgs__msg__RegionOfInterestArray));
  bool success = cs7630_msgs__msg__RegionOfInterestArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cs7630_msgs__msg__RegionOfInterestArray__destroy(cs7630_msgs__msg__RegionOfInterestArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cs7630_msgs__msg__RegionOfInterestArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cs7630_msgs__msg__RegionOfInterestArray__Sequence__init(cs7630_msgs__msg__RegionOfInterestArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cs7630_msgs__msg__RegionOfInterestArray * data = NULL;

  if (size) {
    data = (cs7630_msgs__msg__RegionOfInterestArray *)allocator.zero_allocate(size, sizeof(cs7630_msgs__msg__RegionOfInterestArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cs7630_msgs__msg__RegionOfInterestArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cs7630_msgs__msg__RegionOfInterestArray__fini(&data[i - 1]);
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
cs7630_msgs__msg__RegionOfInterestArray__Sequence__fini(cs7630_msgs__msg__RegionOfInterestArray__Sequence * array)
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
      cs7630_msgs__msg__RegionOfInterestArray__fini(&array->data[i]);
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

cs7630_msgs__msg__RegionOfInterestArray__Sequence *
cs7630_msgs__msg__RegionOfInterestArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cs7630_msgs__msg__RegionOfInterestArray__Sequence * array = (cs7630_msgs__msg__RegionOfInterestArray__Sequence *)allocator.allocate(sizeof(cs7630_msgs__msg__RegionOfInterestArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cs7630_msgs__msg__RegionOfInterestArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cs7630_msgs__msg__RegionOfInterestArray__Sequence__destroy(cs7630_msgs__msg__RegionOfInterestArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cs7630_msgs__msg__RegionOfInterestArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cs7630_msgs__msg__RegionOfInterestArray__Sequence__are_equal(const cs7630_msgs__msg__RegionOfInterestArray__Sequence * lhs, const cs7630_msgs__msg__RegionOfInterestArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cs7630_msgs__msg__RegionOfInterestArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cs7630_msgs__msg__RegionOfInterestArray__Sequence__copy(
  const cs7630_msgs__msg__RegionOfInterestArray__Sequence * input,
  cs7630_msgs__msg__RegionOfInterestArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cs7630_msgs__msg__RegionOfInterestArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cs7630_msgs__msg__RegionOfInterestArray * data =
      (cs7630_msgs__msg__RegionOfInterestArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cs7630_msgs__msg__RegionOfInterestArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cs7630_msgs__msg__RegionOfInterestArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cs7630_msgs__msg__RegionOfInterestArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
