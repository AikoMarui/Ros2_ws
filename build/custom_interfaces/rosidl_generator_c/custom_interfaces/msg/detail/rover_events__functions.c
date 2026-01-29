// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interfaces:msg/RoverEvents.idl
// generated code does not contain a copyright notice
#include "custom_interfaces/msg/detail/rover_events__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `info`
#include "std_msgs/msg/detail/string__functions.h"
// Member `rover_location`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
custom_interfaces__msg__RoverEvents__init(custom_interfaces__msg__RoverEvents * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!std_msgs__msg__String__init(&msg->info)) {
    custom_interfaces__msg__RoverEvents__fini(msg);
    return false;
  }
  // rover_location
  if (!geometry_msgs__msg__Pose__init(&msg->rover_location)) {
    custom_interfaces__msg__RoverEvents__fini(msg);
    return false;
  }
  return true;
}

void
custom_interfaces__msg__RoverEvents__fini(custom_interfaces__msg__RoverEvents * msg)
{
  if (!msg) {
    return;
  }
  // info
  std_msgs__msg__String__fini(&msg->info);
  // rover_location
  geometry_msgs__msg__Pose__fini(&msg->rover_location);
}

bool
custom_interfaces__msg__RoverEvents__are_equal(const custom_interfaces__msg__RoverEvents * lhs, const custom_interfaces__msg__RoverEvents * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!std_msgs__msg__String__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // rover_location
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->rover_location), &(rhs->rover_location)))
  {
    return false;
  }
  return true;
}

bool
custom_interfaces__msg__RoverEvents__copy(
  const custom_interfaces__msg__RoverEvents * input,
  custom_interfaces__msg__RoverEvents * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!std_msgs__msg__String__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // rover_location
  if (!geometry_msgs__msg__Pose__copy(
      &(input->rover_location), &(output->rover_location)))
  {
    return false;
  }
  return true;
}

custom_interfaces__msg__RoverEvents *
custom_interfaces__msg__RoverEvents__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__RoverEvents * msg = (custom_interfaces__msg__RoverEvents *)allocator.allocate(sizeof(custom_interfaces__msg__RoverEvents), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interfaces__msg__RoverEvents));
  bool success = custom_interfaces__msg__RoverEvents__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interfaces__msg__RoverEvents__destroy(custom_interfaces__msg__RoverEvents * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interfaces__msg__RoverEvents__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interfaces__msg__RoverEvents__Sequence__init(custom_interfaces__msg__RoverEvents__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__RoverEvents * data = NULL;

  if (size) {
    data = (custom_interfaces__msg__RoverEvents *)allocator.zero_allocate(size, sizeof(custom_interfaces__msg__RoverEvents), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interfaces__msg__RoverEvents__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interfaces__msg__RoverEvents__fini(&data[i - 1]);
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
custom_interfaces__msg__RoverEvents__Sequence__fini(custom_interfaces__msg__RoverEvents__Sequence * array)
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
      custom_interfaces__msg__RoverEvents__fini(&array->data[i]);
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

custom_interfaces__msg__RoverEvents__Sequence *
custom_interfaces__msg__RoverEvents__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__RoverEvents__Sequence * array = (custom_interfaces__msg__RoverEvents__Sequence *)allocator.allocate(sizeof(custom_interfaces__msg__RoverEvents__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interfaces__msg__RoverEvents__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interfaces__msg__RoverEvents__Sequence__destroy(custom_interfaces__msg__RoverEvents__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interfaces__msg__RoverEvents__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interfaces__msg__RoverEvents__Sequence__are_equal(const custom_interfaces__msg__RoverEvents__Sequence * lhs, const custom_interfaces__msg__RoverEvents__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interfaces__msg__RoverEvents__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interfaces__msg__RoverEvents__Sequence__copy(
  const custom_interfaces__msg__RoverEvents__Sequence * input,
  custom_interfaces__msg__RoverEvents__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interfaces__msg__RoverEvents);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interfaces__msg__RoverEvents * data =
      (custom_interfaces__msg__RoverEvents *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interfaces__msg__RoverEvents__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interfaces__msg__RoverEvents__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interfaces__msg__RoverEvents__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
