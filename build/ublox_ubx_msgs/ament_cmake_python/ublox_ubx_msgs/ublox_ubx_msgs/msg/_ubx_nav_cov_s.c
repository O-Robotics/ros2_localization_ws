// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXNavCov.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_nav_cov__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_cov__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_nav_cov__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_nav_cov.UBXNavCov", full_classname_dest, 41) == 0);
  }
  ublox_ubx_msgs__msg__UBXNavCov * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // itow
    PyObject * field = PyObject_GetAttrString(_pymsg, "itow");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->itow = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // version
    PyObject * field = PyObject_GetAttrString(_pymsg, "version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // pos_cor_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_cor_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pos_cor_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // vel_cor_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_cor_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->vel_cor_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pos_cov_nn
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_cov_nn");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pos_cov_nn = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pos_cov_ne
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_cov_ne");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pos_cov_ne = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pos_cov_nd
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_cov_nd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pos_cov_nd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pos_cov_ee
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_cov_ee");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pos_cov_ee = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pos_cov_ed
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_cov_ed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pos_cov_ed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pos_cov_dd
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_cov_dd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pos_cov_dd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_cov_nn
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_cov_nn");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_cov_nn = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_cov_ne
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_cov_ne");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_cov_ne = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_cov_nd
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_cov_nd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_cov_nd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_cov_ee
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_cov_ee");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_cov_ee = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_cov_ed
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_cov_ed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_cov_ed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_cov_dd
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_cov_dd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_cov_dd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__ubx_nav_cov__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXNavCov */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_nav_cov");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXNavCov");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXNavCov * ros_message = (ublox_ubx_msgs__msg__UBXNavCov *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // itow
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->itow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "itow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_cor_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pos_cor_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_cor_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_cor_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->vel_cor_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_cor_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_cov_nn
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pos_cov_nn);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_cov_nn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_cov_ne
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pos_cov_ne);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_cov_ne", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_cov_nd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pos_cov_nd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_cov_nd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_cov_ee
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pos_cov_ee);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_cov_ee", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_cov_ed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pos_cov_ed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_cov_ed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_cov_dd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pos_cov_dd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_cov_dd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_cov_nn
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_cov_nn);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_cov_nn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_cov_ne
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_cov_ne);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_cov_ne", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_cov_nd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_cov_nd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_cov_nd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_cov_ee
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_cov_ee);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_cov_ee", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_cov_ed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_cov_ed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_cov_ed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_cov_dd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_cov_dd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_cov_dd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
