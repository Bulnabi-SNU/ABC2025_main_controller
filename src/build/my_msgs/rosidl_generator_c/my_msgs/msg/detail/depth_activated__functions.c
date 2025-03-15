// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_msgs:msg/DepthActivated.idl
// generated code does not contain a copyright notice
#include "my_msgs/msg/detail/depth_activated__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
my_msgs__msg__DepthActivated__init(my_msgs__msg__DepthActivated * msg)
{
  if (!msg) {
    return false;
  }
  // depth_activated
  return true;
}

void
my_msgs__msg__DepthActivated__fini(my_msgs__msg__DepthActivated * msg)
{
  if (!msg) {
    return;
  }
  // depth_activated
}

bool
my_msgs__msg__DepthActivated__are_equal(const my_msgs__msg__DepthActivated * lhs, const my_msgs__msg__DepthActivated * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // depth_activated
  if (lhs->depth_activated != rhs->depth_activated) {
    return false;
  }
  return true;
}

bool
my_msgs__msg__DepthActivated__copy(
  const my_msgs__msg__DepthActivated * input,
  my_msgs__msg__DepthActivated * output)
{
  if (!input || !output) {
    return false;
  }
  // depth_activated
  output->depth_activated = input->depth_activated;
  return true;
}

my_msgs__msg__DepthActivated *
my_msgs__msg__DepthActivated__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__DepthActivated * msg = (my_msgs__msg__DepthActivated *)allocator.allocate(sizeof(my_msgs__msg__DepthActivated), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_msgs__msg__DepthActivated));
  bool success = my_msgs__msg__DepthActivated__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_msgs__msg__DepthActivated__destroy(my_msgs__msg__DepthActivated * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_msgs__msg__DepthActivated__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_msgs__msg__DepthActivated__Sequence__init(my_msgs__msg__DepthActivated__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__DepthActivated * data = NULL;

  if (size) {
    data = (my_msgs__msg__DepthActivated *)allocator.zero_allocate(size, sizeof(my_msgs__msg__DepthActivated), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_msgs__msg__DepthActivated__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_msgs__msg__DepthActivated__fini(&data[i - 1]);
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
my_msgs__msg__DepthActivated__Sequence__fini(my_msgs__msg__DepthActivated__Sequence * array)
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
      my_msgs__msg__DepthActivated__fini(&array->data[i]);
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

my_msgs__msg__DepthActivated__Sequence *
my_msgs__msg__DepthActivated__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__DepthActivated__Sequence * array = (my_msgs__msg__DepthActivated__Sequence *)allocator.allocate(sizeof(my_msgs__msg__DepthActivated__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_msgs__msg__DepthActivated__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_msgs__msg__DepthActivated__Sequence__destroy(my_msgs__msg__DepthActivated__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_msgs__msg__DepthActivated__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_msgs__msg__DepthActivated__Sequence__are_equal(const my_msgs__msg__DepthActivated__Sequence * lhs, const my_msgs__msg__DepthActivated__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_msgs__msg__DepthActivated__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_msgs__msg__DepthActivated__Sequence__copy(
  const my_msgs__msg__DepthActivated__Sequence * input,
  my_msgs__msg__DepthActivated__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_msgs__msg__DepthActivated);
    my_msgs__msg__DepthActivated * data =
      (my_msgs__msg__DepthActivated *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_msgs__msg__DepthActivated__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          my_msgs__msg__DepthActivated__fini(&data[i]);
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
    if (!my_msgs__msg__DepthActivated__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
