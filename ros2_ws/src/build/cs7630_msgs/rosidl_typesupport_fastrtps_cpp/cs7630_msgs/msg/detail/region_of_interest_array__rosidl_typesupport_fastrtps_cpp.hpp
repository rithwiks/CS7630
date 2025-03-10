// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from cs7630_msgs:msg/RegionOfInterestArray.idl
// generated code does not contain a copyright notice

#ifndef CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "cs7630_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "cs7630_msgs/msg/detail/region_of_interest_array__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace cs7630_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cs7630_msgs
cdr_serialize(
  const cs7630_msgs::msg::RegionOfInterestArray & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cs7630_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  cs7630_msgs::msg::RegionOfInterestArray & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cs7630_msgs
get_serialized_size(
  const cs7630_msgs::msg::RegionOfInterestArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cs7630_msgs
max_serialized_size_RegionOfInterestArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace cs7630_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cs7630_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cs7630_msgs, msg, RegionOfInterestArray)();

#ifdef __cplusplus
}
#endif

#endif  // CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
