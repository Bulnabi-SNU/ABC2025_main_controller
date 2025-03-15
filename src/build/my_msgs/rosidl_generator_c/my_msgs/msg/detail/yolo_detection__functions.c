// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_msgs:msg/YoloDetection.idl
// generated code does not contain a copyright notice
#include "my_msgs/msg/detail/yolo_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `label`
#include "rosidl_runtime_c/string_functions.h"

bool
my_msgs__msg__YoloDetection__init(my_msgs__msg__YoloDetection * msg)
{
  if (!msg) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    my_msgs__msg__YoloDetection__fini(msg);
    return false;
  }
  // screen_width
  // screen_height
  // xmax
  // ymax
  // xmin
  // ymin
  return true;
}

void
my_msgs__msg__YoloDetection__fini(my_msgs__msg__YoloDetection * msg)
{
  if (!msg) {
    return;
  }
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // screen_width
  // screen_height
  // xmax
  // ymax
  // xmin
  // ymin
}

bool
my_msgs__msg__YoloDetection__are_equal(const my_msgs__msg__YoloDetection * lhs, const my_msgs__msg__YoloDetection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // screen_width
  if (lhs->screen_width != rhs->screen_width) {
    return false;
  }
  // screen_height
  if (lhs->screen_height != rhs->screen_height) {
    return false;
  }
  // xmax
  if (lhs->xmax != rhs->xmax) {
    return false;
  }
  // ymax
  if (lhs->ymax != rhs->ymax) {
    return false;
  }
  // xmin
  if (lhs->xmin != rhs->xmin) {
    return false;
  }
  // ymin
  if (lhs->ymin != rhs->ymin) {
    return false;
  }
  return true;
}

bool
my_msgs__msg__YoloDetection__copy(
  const my_msgs__msg__YoloDetection * input,
  my_msgs__msg__YoloDetection * output)
{
  if (!input || !output) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // screen_width
  output->screen_width = input->screen_width;
  // screen_height
  output->screen_height = input->screen_height;
  // xmax
  output->xmax = input->xmax;
  // ymax
  output->ymax = input->ymax;
  // xmin
  output->xmin = input->xmin;
  // ymin
  output->ymin = input->ymin;
  return true;
}

my_msgs__msg__YoloDetection *
my_msgs__msg__YoloDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__YoloDetection * msg = (my_msgs__msg__YoloDetection *)allocator.allocate(sizeof(my_msgs__msg__YoloDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_msgs__msg__YoloDetection));
  bool success = my_msgs__msg__YoloDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_msgs__msg__YoloDetection__destroy(my_msgs__msg__YoloDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_msgs__msg__YoloDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_msgs__msg__YoloDetection__Sequence__init(my_msgs__msg__YoloDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__YoloDetection * data = NULL;

  if (size) {
    data = (my_msgs__msg__YoloDetection *)allocator.zero_allocate(size, sizeof(my_msgs__msg__YoloDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_msgs__msg__YoloDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_msgs__msg__YoloDetection__fini(&data[i - 1]);
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
my_msgs__msg__YoloDetection__Sequence__fini(my_msgs__msg__YoloDetection__Sequence * array)
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
      my_msgs__msg__YoloDetection__fini(&array->data[i]);
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

my_msgs__msg__YoloDetection__Sequence *
my_msgs__msg__YoloDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__YoloDetection__Sequence * array = (my_msgs__msg__YoloDetection__Sequence *)allocator.allocate(sizeof(my_msgs__msg__YoloDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_msgs__msg__YoloDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_msgs__msg__YoloDetection__Sequence__destroy(my_msgs__msg__YoloDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_msgs__msg__YoloDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_msgs__msg__YoloDetection__Sequence__are_equal(const my_msgs__msg__YoloDetection__Sequence * lhs, const my_msgs__msg__YoloDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_msgs__msg__YoloDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_msgs__msg__YoloDetection__Sequence__copy(
  const my_msgs__msg__YoloDetection__Sequence * input,
  my_msgs__msg__YoloDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_msgs__msg__YoloDetection);
    my_msgs__msg__YoloDetection * data =
      (my_msgs__msg__YoloDetection *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_msgs__msg__YoloDetection__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          my_msgs__msg__YoloDetection__fini(&data[i]);
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
    if (!my_msgs__msg__YoloDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
