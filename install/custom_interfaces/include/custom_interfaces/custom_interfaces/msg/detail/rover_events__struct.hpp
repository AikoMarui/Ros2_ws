// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/RoverEvents.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'info'
#include "std_msgs/msg/detail/string__struct.hpp"
// Member 'rover_location'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__RoverEvents __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__RoverEvents __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RoverEvents_
{
  using Type = RoverEvents_<ContainerAllocator>;

  explicit RoverEvents_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init),
    rover_location(_init)
  {
    (void)_init;
  }

  explicit RoverEvents_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init),
    rover_location(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _info_type info;
  using _rover_location_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _rover_location_type rover_location;

  // setters for named parameter idiom
  Type & set__info(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__rover_location(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->rover_location = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::RoverEvents_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::RoverEvents_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::RoverEvents_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::RoverEvents_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::RoverEvents_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::RoverEvents_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::RoverEvents_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::RoverEvents_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::RoverEvents_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::RoverEvents_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__RoverEvents
    std::shared_ptr<custom_interfaces::msg::RoverEvents_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__RoverEvents
    std::shared_ptr<custom_interfaces::msg::RoverEvents_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RoverEvents_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->rover_location != other.rover_location) {
      return false;
    }
    return true;
  }
  bool operator!=(const RoverEvents_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RoverEvents_

// alias to use template instance with default allocator
using RoverEvents =
  custom_interfaces::msg::RoverEvents_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__STRUCT_HPP_
