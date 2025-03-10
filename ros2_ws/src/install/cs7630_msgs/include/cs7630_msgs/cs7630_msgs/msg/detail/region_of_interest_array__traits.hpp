// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cs7630_msgs:msg/RegionOfInterestArray.idl
// generated code does not contain a copyright notice

#ifndef CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__TRAITS_HPP_
#define CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cs7630_msgs/msg/detail/region_of_interest_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'regions'
#include "sensor_msgs/msg/detail/region_of_interest__traits.hpp"

namespace cs7630_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RegionOfInterestArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: regions
  {
    if (msg.regions.size() == 0) {
      out << "regions: []";
    } else {
      out << "regions: [";
      size_t pending_items = msg.regions.size();
      for (auto item : msg.regions) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RegionOfInterestArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: regions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.regions.size() == 0) {
      out << "regions: []\n";
    } else {
      out << "regions:\n";
      for (auto item : msg.regions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RegionOfInterestArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace cs7630_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cs7630_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cs7630_msgs::msg::RegionOfInterestArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  cs7630_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cs7630_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cs7630_msgs::msg::RegionOfInterestArray & msg)
{
  return cs7630_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cs7630_msgs::msg::RegionOfInterestArray>()
{
  return "cs7630_msgs::msg::RegionOfInterestArray";
}

template<>
inline const char * name<cs7630_msgs::msg::RegionOfInterestArray>()
{
  return "cs7630_msgs/msg/RegionOfInterestArray";
}

template<>
struct has_fixed_size<cs7630_msgs::msg::RegionOfInterestArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cs7630_msgs::msg::RegionOfInterestArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cs7630_msgs::msg::RegionOfInterestArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__TRAITS_HPP_
