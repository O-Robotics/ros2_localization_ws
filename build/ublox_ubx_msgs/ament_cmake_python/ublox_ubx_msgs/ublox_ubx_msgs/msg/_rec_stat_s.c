// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/RecStat.idl
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
#include "ublox_ubx_msgs/msg/detail/rec_stat__struct.h"
#include "ublox_ubx_msgs/msg/detail/rec_stat__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__rec_stat__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[37];
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
    assert(strncmp("ublox_ubx_msgs.msg._rec_stat.RecStat", full_classname_dest, 36) == 0);
  }
  ublox_ubx_msgs__msg__RecStat * ros_message = _ros_message;
  {  // leap_sec
    PyObject * field = PyObject_GetAttrString(_pymsg, "leap_sec");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->leap_sec = (Py_True == field);
    Py_DECREF(field);
  }
  {  // clk_reset
    PyObject * field = PyObject_GetAttrString(_pymsg, "clk_reset");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->clk_reset = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__rec_stat__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RecStat */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._rec_stat");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RecStat");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__RecStat * ros_message = (ublox_ubx_msgs__msg__RecStat *)raw_ros_message;
  {  // leap_sec
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->leap_sec ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "leap_sec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // clk_reset
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->clk_reset ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "clk_reset", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
