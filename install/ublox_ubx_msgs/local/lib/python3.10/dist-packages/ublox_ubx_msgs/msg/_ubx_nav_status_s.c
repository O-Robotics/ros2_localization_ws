// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXNavStatus.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__gps_fix__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__gps_fix__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__map_matching__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__map_matching__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__psm_status__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__psm_status__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__spoof_det__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__spoof_det__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__carr_soln__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__carr_soln__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_nav_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_nav_status.UBXNavStatus", full_classname_dest, 47) == 0);
  }
  ublox_ubx_msgs__msg__UBXNavStatus * ros_message = _ros_message;
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
  {  // gps_fix
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_fix");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__gps_fix__convert_from_py(field, &ros_message->gps_fix)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // gps_fix_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_fix_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->gps_fix_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // diff_soln
    PyObject * field = PyObject_GetAttrString(_pymsg, "diff_soln");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->diff_soln = (Py_True == field);
    Py_DECREF(field);
  }
  {  // wkn_set
    PyObject * field = PyObject_GetAttrString(_pymsg, "wkn_set");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->wkn_set = (Py_True == field);
    Py_DECREF(field);
  }
  {  // tow_set
    PyObject * field = PyObject_GetAttrString(_pymsg, "tow_set");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->tow_set = (Py_True == field);
    Py_DECREF(field);
  }
  {  // diff_corr
    PyObject * field = PyObject_GetAttrString(_pymsg, "diff_corr");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->diff_corr = (Py_True == field);
    Py_DECREF(field);
  }
  {  // carr_soln_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "carr_soln_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->carr_soln_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // map_matching
    PyObject * field = PyObject_GetAttrString(_pymsg, "map_matching");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__map_matching__convert_from_py(field, &ros_message->map_matching)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // psm
    PyObject * field = PyObject_GetAttrString(_pymsg, "psm");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__psm_status__convert_from_py(field, &ros_message->psm)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // spoof_det
    PyObject * field = PyObject_GetAttrString(_pymsg, "spoof_det");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__spoof_det__convert_from_py(field, &ros_message->spoof_det)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // carr_soln
    PyObject * field = PyObject_GetAttrString(_pymsg, "carr_soln");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__carr_soln__convert_from_py(field, &ros_message->carr_soln)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // ttff
    PyObject * field = PyObject_GetAttrString(_pymsg, "ttff");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ttff = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // msss
    PyObject * field = PyObject_GetAttrString(_pymsg, "msss");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->msss = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__ubx_nav_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXNavStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_nav_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXNavStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXNavStatus * ros_message = (ublox_ubx_msgs__msg__UBXNavStatus *)raw_ros_message;
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
  {  // gps_fix
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__gps_fix__convert_to_py(&ros_message->gps_fix);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_fix", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gps_fix_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->gps_fix_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_fix_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // diff_soln
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->diff_soln ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "diff_soln", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wkn_set
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->wkn_set ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wkn_set", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tow_set
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->tow_set ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tow_set", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // diff_corr
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->diff_corr ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "diff_corr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // carr_soln_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->carr_soln_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "carr_soln_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // map_matching
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__map_matching__convert_to_py(&ros_message->map_matching);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "map_matching", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // psm
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__psm_status__convert_to_py(&ros_message->psm);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "psm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // spoof_det
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__spoof_det__convert_to_py(&ros_message->spoof_det);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "spoof_det", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // carr_soln
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__carr_soln__convert_to_py(&ros_message->carr_soln);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "carr_soln", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ttff
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ttff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ttff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // msss
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->msss);
    {
      int rc = PyObject_SetAttrString(_pymessage, "msss", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
