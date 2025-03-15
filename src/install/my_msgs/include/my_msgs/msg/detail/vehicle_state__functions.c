// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice
#include "my_msgs/msg/detail/vehicle_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `state`
// Member `substate`
#include "rosidl_runtime_c/string_functions.h"

bool
my_msgs__msg__VehicleState__init(my_msgs__msg__VehicleState * msg)
{
  if (!msg) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    my_msgs__msg__VehicleState__fini(msg);
    return false;
  }
  // substate
  if (!rosidl_runtime_c__String__init(&msg->substate)) {
    my_msgs__msg__VehicleState__fini(msg);
    return false;
  }
  return true;
}

void
my_msgs__msg__VehicleState__fini(my_msgs__msg__VehicleState * msg)
{
  if (!msg) {
    return;
  }
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // substate
  rosidl_runtime_c__String__fini(&msg->substate);
}

bool
my_msgs__msg__VehicleState__are_equal(const my_msgs__msg__VehicleState * lhs, const my_msgs__msg__VehicleState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // substate
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->substate), &(rhs->substate)))
  {
    return false;
  }
  return true;
}

bool
my_msgs__msg__VehicleState__copy(
  const my_msgs__msg__VehicleState * input,
  my_msgs__msg__VehicleState * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // substate
  if (!rosidl_runtime_c__String__copy(
      &(input->substate), &(output->substate)))
  {
    return false;
  }
  return true;
}

my_msgs__msg__VehicleState *
my_msgs__msg__VehicleState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__VehicleState * msg = (my_msgs__msg__VehicleState *)allocator.allocate(sizeof(my_msgs__msg__VehicleState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_msgs__msg__VehicleState));
  bool success = my_msgs__msg__VehicleState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_msgs__msg__VehicleState__destroy(my_msgs__msg__VehicleState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_msgs__msg__VehicleState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_msgs__msg__VehicleState__Sequence__init(my_msgs__msg__VehicleState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__VehicleState * data = NULL;

  if (size) {
    data = (my_msgs__msg__VehicleState *)allocator.zero_allocate(size, sizeof(my_msgs__msg__VehicleState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_msgs__msg__VehicleState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_msgs__msg__VehicleState__fini(&data[i - 1]);
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
my_msgs__msg__VehicleState__Sequence__fini(my_msgs__msg__VehicleState__Sequence * array)
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
      my_msgs__msg__VehicleState__fini(&array->data[i]);
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

my_msgs__msg__VehicleState__Sequence *
my_msgs__msg__VehicleState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__VehicleState__Sequence * array = (my_msgs__msg__VehicleState__Sequence *)allocator.allocate(sizeof(my_msgs__msg__VehicleState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_msgs__msg__VehicleState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_msgs__msg__VehicleState__Sequence__destroy(my_msgs__msg__VehicleState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_msgs__msg__VehicleState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_msgs__msg__VehicleState__Sequence__are_equal(const my_msgs__msg__VehicleState__Sequence * lhs, const my_msgs__msg__VehicleState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_msgs__msg__VehicleState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_msgs__msg__VehicleState__Sequence__copy(
  const my_msgs__msg__VehicleState__Sequence * input,
  my_msgs__msg__VehicleState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_msgs__msg__VehicleState);
    my_msgs__msg__VehicleState * data =
      (my_msgs__msg__VehicleState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_msgs__msg__VehicleState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          my_msgs__msg__VehicleState__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_msgs__msg__VehicleState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
