// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/SBASService.idl
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
#include "ublox_ubx_msgs/msg/detail/sbas_service__struct.h"
#include "ublox_ubx_msgs/msg/detail/sbas_service__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__sbas_service__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
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
    assert(strncmp("ublox_ubx_msgs.msg._sbas_service.SBASService", full_classname_dest, 44) == 0);
  }
  ublox_ubx_msgs__msg__SBASService * ros_message = _ros_message;
  {  // ranging
    PyObject * field = PyObject_GetAttrString(_pymsg, "ranging");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->ranging = (Py_True == field);
    Py_DECREF(field);
  }
  {  // corrections
    PyObject * field = PyObject_GetAttrString(_pymsg, "corrections");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->corrections = (Py_True == field);
    Py_DECREF(field);
  }
  {  // integrity
    PyObject * field = PyObject_GetAttrString(_pymsg, "integrity");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->integrity = (Py_True == field);
    Py_DECREF(field);
  }
  {  // test_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "test_mode");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->test_mode = (Py_True == field);
    Py_DECREF(field);
  }
  {  // bad
    PyObject * field = PyObject_GetAttrString(_pymsg, "bad");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->bad = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__sbas_service__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SBASService */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._sbas_service");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SBASService");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__SBASService * ros_message = (ublox_ubx_msgs__msg__SBASService *)raw_ros_message;
  {  // ranging
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->ranging ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ranging", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // corrections
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->corrections ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "corrections", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // integrity
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->integrity ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "integrity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // test_mode
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->test_mode ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "test_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bad
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->bad ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
