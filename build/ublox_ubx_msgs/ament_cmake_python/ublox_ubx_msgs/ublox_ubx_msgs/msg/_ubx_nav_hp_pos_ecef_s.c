// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosECEF.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_hp_pos_ecef__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_nav_hp_pos_ecef__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_nav_hp_pos_ecef.UBXNavHPPosECEF", full_classname_dest, 55) == 0);
  }
  ublox_ubx_msgs__msg__UBXNavHPPosECEF * ros_message = _ros_message;
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
  {  // version
    PyObject * field = PyObject_GetAttrString(_pymsg, "version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version = (uint8_t)PyLong_AsUnsignedLong(field);
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
  {  // ecef_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "ecef_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ecef_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // ecef_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "ecef_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ecef_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // ecef_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "ecef_z");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ecef_z = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // ecef_x_hp
    PyObject * field = PyObject_GetAttrString(_pymsg, "ecef_x_hp");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ecef_x_hp = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // ecef_y_hp
    PyObject * field = PyObject_GetAttrString(_pymsg, "ecef_y_hp");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ecef_y_hp = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // ecef_z_hp
    PyObject * field = PyObject_GetAttrString(_pymsg, "ecef_z_hp");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ecef_z_hp = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // invalid_ecef_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "invalid_ecef_x");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->invalid_ecef_x = (Py_True == field);
    Py_DECREF(field);
  }
  {  // invalid_ecef_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "invalid_ecef_y");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->invalid_ecef_y = (Py_True == field);
    Py_DECREF(field);
  }
  {  // invalid_ecef_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "invalid_ecef_z");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->invalid_ecef_z = (Py_True == field);
    Py_DECREF(field);
  }
  {  // invalid_ecef_x_hp
    PyObject * field = PyObject_GetAttrString(_pymsg, "invalid_ecef_x_hp");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->invalid_ecef_x_hp = (Py_True == field);
    Py_DECREF(field);
  }
  {  // invalid_ecef_y_hp
    PyObject * field = PyObject_GetAttrString(_pymsg, "invalid_ecef_y_hp");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->invalid_ecef_y_hp = (Py_True == field);
    Py_DECREF(field);
  }
  {  // invalid_ecef_z_hp
    PyObject * field = PyObject_GetAttrString(_pymsg, "invalid_ecef_z_hp");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->invalid_ecef_z_hp = (Py_True == field);
    Py_DECREF(field);
  }
  {  // p_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->p_acc = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__ubx_nav_hp_pos_ecef__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXNavHPPosECEF */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_nav_hp_pos_ecef");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXNavHPPosECEF");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXNavHPPosECEF * ros_message = (ublox_ubx_msgs__msg__UBXNavHPPosECEF *)raw_ros_message;
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
  {  // ecef_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->ecef_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ecef_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ecef_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->ecef_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ecef_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ecef_z
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->ecef_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ecef_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ecef_x_hp
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->ecef_x_hp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ecef_x_hp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ecef_y_hp
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->ecef_y_hp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ecef_y_hp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ecef_z_hp
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->ecef_z_hp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ecef_z_hp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // invalid_ecef_x
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->invalid_ecef_x ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "invalid_ecef_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // invalid_ecef_y
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->invalid_ecef_y ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "invalid_ecef_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // invalid_ecef_z
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->invalid_ecef_z ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "invalid_ecef_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // invalid_ecef_x_hp
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->invalid_ecef_x_hp ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "invalid_ecef_x_hp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // invalid_ecef_y_hp
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->invalid_ecef_y_hp ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "invalid_ecef_y_hp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // invalid_ecef_z_hp
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->invalid_ecef_z_hp ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "invalid_ecef_z_hp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // p_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->p_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
