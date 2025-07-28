// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXNavTimeUTC.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_nav_time_utc__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_time_utc__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__utc_std__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__utc_std__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_nav_time_utc__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_nav_time_utc.UBXNavTimeUTC", full_classname_dest, 50) == 0);
  }
  ublox_ubx_msgs__msg__UBXNavTimeUTC * ros_message = _ros_message;
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
  {  // t_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->t_acc = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // nano
    PyObject * field = PyObject_GetAttrString(_pymsg, "nano");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->nano = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // year
    PyObject * field = PyObject_GetAttrString(_pymsg, "year");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->year = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // month
    PyObject * field = PyObject_GetAttrString(_pymsg, "month");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->month = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // day
    PyObject * field = PyObject_GetAttrString(_pymsg, "day");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->day = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // hour
    PyObject * field = PyObject_GetAttrString(_pymsg, "hour");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->hour = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // min
    PyObject * field = PyObject_GetAttrString(_pymsg, "min");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->min = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // sec
    PyObject * field = PyObject_GetAttrString(_pymsg, "sec");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sec = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // valid_tow
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid_tow");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid_tow = (Py_True == field);
    Py_DECREF(field);
  }
  {  // valid_wkn
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid_wkn");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid_wkn = (Py_True == field);
    Py_DECREF(field);
  }
  {  // valid_utc
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid_utc");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid_utc = (Py_True == field);
    Py_DECREF(field);
  }
  {  // utc_std
    PyObject * field = PyObject_GetAttrString(_pymsg, "utc_std");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__utc_std__convert_from_py(field, &ros_message->utc_std)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__ubx_nav_time_utc__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXNavTimeUTC */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_nav_time_utc");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXNavTimeUTC");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXNavTimeUTC * ros_message = (ublox_ubx_msgs__msg__UBXNavTimeUTC *)raw_ros_message;
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
  {  // t_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->t_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // nano
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->nano);
    {
      int rc = PyObject_SetAttrString(_pymessage, "nano", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // year
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->year);
    {
      int rc = PyObject_SetAttrString(_pymessage, "year", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // month
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->month);
    {
      int rc = PyObject_SetAttrString(_pymessage, "month", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // day
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->day);
    {
      int rc = PyObject_SetAttrString(_pymessage, "day", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hour
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->hour);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hour", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sec
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->sec);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid_tow
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid_tow ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid_tow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid_wkn
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid_wkn ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid_wkn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid_utc
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid_utc ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid_utc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // utc_std
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__utc_std__convert_to_py(&ros_message->utc_std);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "utc_std", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
