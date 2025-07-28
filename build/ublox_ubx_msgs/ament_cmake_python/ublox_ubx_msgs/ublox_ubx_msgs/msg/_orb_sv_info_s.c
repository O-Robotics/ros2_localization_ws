// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/OrbSVInfo.idl
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
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__struct.h"
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__functions.h"

bool ublox_ubx_msgs__msg__orb_sv_flag__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__orb_sv_flag__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__orb_eph_info__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__orb_eph_info__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__orb_alm_info__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__orb_alm_info__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__other_orb_info__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__other_orb_info__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__orb_sv_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ublox_ubx_msgs.msg._orb_sv_info.OrbSVInfo", full_classname_dest, 41) == 0);
  }
  ublox_ubx_msgs__msg__OrbSVInfo * ros_message = _ros_message;
  {  // gnss_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "gnss_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->gnss_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sv_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "sv_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sv_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sv_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "sv_flag");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__orb_sv_flag__convert_from_py(field, &ros_message->sv_flag)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // eph
    PyObject * field = PyObject_GetAttrString(_pymsg, "eph");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__orb_eph_info__convert_from_py(field, &ros_message->eph)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // alm
    PyObject * field = PyObject_GetAttrString(_pymsg, "alm");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__orb_alm_info__convert_from_py(field, &ros_message->alm)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // other_orb
    PyObject * field = PyObject_GetAttrString(_pymsg, "other_orb");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__other_orb_info__convert_from_py(field, &ros_message->other_orb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__orb_sv_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OrbSVInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._orb_sv_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OrbSVInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__OrbSVInfo * ros_message = (ublox_ubx_msgs__msg__OrbSVInfo *)raw_ros_message;
  {  // gnss_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->gnss_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gnss_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sv_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sv_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sv_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sv_flag
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__orb_sv_flag__convert_to_py(&ros_message->sv_flag);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "sv_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // eph
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__orb_eph_info__convert_to_py(&ros_message->eph);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "eph", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // alm
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__orb_alm_info__convert_to_py(&ros_message->alm);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "alm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // other_orb
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__other_orb_info__convert_to_py(&ros_message->other_orb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "other_orb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
