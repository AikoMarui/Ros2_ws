// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:msg/RoverEvents.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__TRAITS_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/msg/detail/rover_events__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'info'
#include "std_msgs/msg/detail/string__traits.hpp"
// Member 'rover_location'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RoverEvents & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: rover_location
  {
    out << "rover_location: ";
    to_flow_style_yaml(msg.rover_location, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RoverEvents & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: rover_location
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rover_location:\n";
    to_block_style_yaml(msg.rover_location, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RoverEvents & msg, bool use_flow_style = false)
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

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::msg::RoverEvents & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::msg::RoverEvents & msg)
{
  return custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::msg::RoverEvents>()
{
  return "custom_interfaces::msg::RoverEvents";
}

template<>
inline const char * name<custom_interfaces::msg::RoverEvents>()
{
  return "custom_interfaces/msg/RoverEvents";
}

template<>
struct has_fixed_size<custom_interfaces::msg::RoverEvents>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<std_msgs::msg::String>::value> {};

template<>
struct has_bounded_size<custom_interfaces::msg::RoverEvents>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<std_msgs::msg::String>::value> {};

template<>
struct is_message<custom_interfaces::msg::RoverEvents>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__TRAITS_HPP_
