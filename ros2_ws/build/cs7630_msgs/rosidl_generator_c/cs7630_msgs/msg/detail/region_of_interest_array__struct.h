// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cs7630_msgs:msg/RegionOfInterestArray.idl
// generated code does not contain a copyright notice

#ifndef CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__STRUCT_H_
#define CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'regions'
#include "sensor_msgs/msg/detail/region_of_interest__struct.h"

/// Struct defined in msg/RegionOfInterestArray in the package cs7630_msgs.
typedef struct cs7630_msgs__msg__RegionOfInterestArray
{
  sensor_msgs__msg__RegionOfInterest__Sequence regions;
} cs7630_msgs__msg__RegionOfInterestArray;

// Struct for a sequence of cs7630_msgs__msg__RegionOfInterestArray.
typedef struct cs7630_msgs__msg__RegionOfInterestArray__Sequence
{
  cs7630_msgs__msg__RegionOfInterestArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cs7630_msgs__msg__RegionOfInterestArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__STRUCT_H_
