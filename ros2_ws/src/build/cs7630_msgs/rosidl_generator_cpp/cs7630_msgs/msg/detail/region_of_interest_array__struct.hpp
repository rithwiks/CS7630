// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cs7630_msgs:msg/RegionOfInterestArray.idl
// generated code does not contain a copyright notice

#ifndef CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__STRUCT_HPP_
#define CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'regions'
#include "sensor_msgs/msg/detail/region_of_interest__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cs7630_msgs__msg__RegionOfInterestArray __attribute__((deprecated))
#else
# define DEPRECATED__cs7630_msgs__msg__RegionOfInterestArray __declspec(deprecated)
#endif

namespace cs7630_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RegionOfInterestArray_
{
  using Type = RegionOfInterestArray_<ContainerAllocator>;

  explicit RegionOfInterestArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit RegionOfInterestArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _regions_type =
    std::vector<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>>>;
  _regions_type regions;

  // setters for named parameter idiom
  Type & set__regions(
    const std::vector<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>>> & _arg)
  {
    this->regions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cs7630_msgs__msg__RegionOfInterestArray
    std::shared_ptr<cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cs7630_msgs__msg__RegionOfInterestArray
    std::shared_ptr<cs7630_msgs::msg::RegionOfInterestArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RegionOfInterestArray_ & other) const
  {
    if (this->regions != other.regions) {
      return false;
    }
    return true;
  }
  bool operator!=(const RegionOfInterestArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RegionOfInterestArray_

// alias to use template instance with default allocator
using RegionOfInterestArray =
  cs7630_msgs::msg::RegionOfInterestArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cs7630_msgs

#endif  // CS7630_MSGS__MSG__DETAIL__REGION_OF_INTEREST_ARRAY__STRUCT_HPP_
