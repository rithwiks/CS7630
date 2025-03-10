// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cs7630_msgs:msg/RegionOfInterestArray.idl
// generated code does not contain a copyright notice

#ifndef CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__BUILDER_HPP_
#define CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cs7630_msgs/msg/detail/region_of_interest_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cs7630_msgs
{

namespace msg
{

namespace builder
{

class Init_RegionOfInterestArray_regions
{
public:
  Init_RegionOfInterestArray_regions()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cs7630_msgs::msg::RegionOfInterestArray regions(::cs7630_msgs::msg::RegionOfInterestArray::_regions_type arg)
  {
    msg_.regions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cs7630_msgs::msg::RegionOfInterestArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cs7630_msgs::msg::RegionOfInterestArray>()
{
  return cs7630_msgs::msg::builder::Init_RegionOfInterestArray_regions();
}

}  // namespace cs7630_msgs

#endif  // CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__BUILDER_HPP_
