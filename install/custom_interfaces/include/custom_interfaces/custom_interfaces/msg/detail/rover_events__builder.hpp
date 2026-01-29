// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/RoverEvents.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/rover_events__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_RoverEvents_rover_location
{
public:
  explicit Init_RoverEvents_rover_location(::custom_interfaces::msg::RoverEvents & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::RoverEvents rover_location(::custom_interfaces::msg::RoverEvents::_rover_location_type arg)
  {
    msg_.rover_location = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::RoverEvents msg_;
};

class Init_RoverEvents_info
{
public:
  Init_RoverEvents_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverEvents_rover_location info(::custom_interfaces::msg::RoverEvents::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RoverEvents_rover_location(msg_);
  }

private:
  ::custom_interfaces::msg::RoverEvents msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::RoverEvents>()
{
  return custom_interfaces::msg::builder::Init_RoverEvents_info();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__BUILDER_HPP_
