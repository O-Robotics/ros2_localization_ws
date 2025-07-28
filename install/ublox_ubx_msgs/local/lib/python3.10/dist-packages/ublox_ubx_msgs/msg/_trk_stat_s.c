// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/TrkStat.idl
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
#include "ublox_ubx_msgs/msg/detail/trk_stat__struct.h"
#include "ublox_ubx_msgs/msg/detail/trk_stat__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__trk_stat__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ublox_ubx_msgs.msg._trk_stat.TrkStat", full_classname_dest, 36) == 0);
  }
  ublox_ubx_msgs__msg__TrkStat * ros_message = _ros_message;
  {  // pr_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "pr_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pr_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // cp_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "cp_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cp_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // half_cyc
    PyObject * field = PyObject_GetAttrString(_pymsg, "half_cyc");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->half_cyc = (Py_True == field);
    Py_DECREF(field);
  }
  {  // sub_half_cyc
    PyObject * field = PyObject_GetAttrString(_pymsg, "sub_half_cyc");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->sub_half_cyc = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__trk_stat__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TrkStat */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._trk_stat");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TrkStat");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__TrkStat * ros_message = (ublox_ubx_msgs__msg__TrkStat *)raw_ros_message;
  {  // pr_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pr_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pr_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cp_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cp_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cp_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // half_cyc
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->half_cyc ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "half_cyc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sub_half_cyc
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->sub_half_cyc ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sub_half_cyc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
