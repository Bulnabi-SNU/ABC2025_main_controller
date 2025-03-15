// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from my_msgs:msg/YoloDetection.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "my_msgs/msg/detail/yolo_detection__struct.h"
#include "my_msgs/msg/detail/yolo_detection__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool my_msgs__msg__yolo_detection__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("my_msgs.msg._yolo_detection.YoloDetection", full_classname_dest, 41) == 0);
  }
  my_msgs__msg__YoloDetection * ros_message = _ros_message;
  {  // label
    PyObject * field = PyObject_GetAttrString(_pymsg, "label");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->label, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // screen_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "screen_width");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->screen_width = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // screen_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "screen_height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->screen_height = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // xmax
    PyObject * field = PyObject_GetAttrString(_pymsg, "xmax");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->xmax = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ymax
    PyObject * field = PyObject_GetAttrString(_pymsg, "ymax");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ymax = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // xmin
    PyObject * field = PyObject_GetAttrString(_pymsg, "xmin");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->xmin = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ymin
    PyObject * field = PyObject_GetAttrString(_pymsg, "ymin");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ymin = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * my_msgs__msg__yolo_detection__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of YoloDetection */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("my_msgs.msg._yolo_detection");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "YoloDetection");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  my_msgs__msg__YoloDetection * ros_message = (my_msgs__msg__YoloDetection *)raw_ros_message;
  {  // label
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->label.data,
      strlen(ros_message->label.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "label", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // screen_width
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->screen_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "screen_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // screen_height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->screen_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "screen_height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // xmax
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->xmax);
    {
      int rc = PyObject_SetAttrString(_pymessage, "xmax", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ymax
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ymax);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ymax", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // xmin
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->xmin);
    {
      int rc = PyObject_SetAttrString(_pymessage, "xmin", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ymin
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ymin);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ymin", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
