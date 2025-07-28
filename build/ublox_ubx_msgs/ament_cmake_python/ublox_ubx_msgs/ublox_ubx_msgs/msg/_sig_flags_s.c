// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/SigFlags.idl
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
#include "ublox_ubx_msgs/msg/detail/sig_flags__struct.h"
#include "ublox_ubx_msgs/msg/detail/sig_flags__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__sig_flags__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
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
    assert(strncmp("ublox_ubx_msgs.msg._sig_flags.SigFlags", full_classname_dest, 38) == 0);
  }
  ublox_ubx_msgs__msg__SigFlags * ros_message = _ros_message;
  {  // health
    PyObject * field = PyObject_GetAttrString(_pymsg, "health");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->health = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // pr_smoothed
    PyObject * field = PyObject_GetAttrString(_pymsg, "pr_smoothed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pr_smoothed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "pr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // cr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "cr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // do_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "do_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->do_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pr_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "pr_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pr_corr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // cr_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "cr_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cr_corr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // do_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "do_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->do_corr_used = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__sig_flags__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SigFlags */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._sig_flags");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SigFlags");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__SigFlags * ros_message = (ublox_ubx_msgs__msg__SigFlags *)raw_ros_message;
  {  // health
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->health);
    {
      int rc = PyObject_SetAttrString(_pymessage, "health", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pr_smoothed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pr_smoothed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pr_smoothed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // do_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->do_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "do_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pr_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pr_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pr_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cr_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cr_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cr_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // do_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->do_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "do_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
