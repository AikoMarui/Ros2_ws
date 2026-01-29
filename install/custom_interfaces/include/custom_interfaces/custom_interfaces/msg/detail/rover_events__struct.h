// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/RoverEvents.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "std_msgs/msg/detail/string__struct.h"
// Member 'rover_location'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/RoverEvents in the package custom_interfaces.
typedef struct custom_interfaces__msg__RoverEvents
{
  /// Informação sobre descoberta, problema ou ação
  std_msgs__msg__String info;
  /// Posição do rover no momento do evento
  geometry_msgs__msg__Pose rover_location;
} custom_interfaces__msg__RoverEvents;

// Struct for a sequence of custom_interfaces__msg__RoverEvents.
typedef struct custom_interfaces__msg__RoverEvents__Sequence
{
  custom_interfaces__msg__RoverEvents * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__RoverEvents__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROVER_EVENTS__STRUCT_H_
