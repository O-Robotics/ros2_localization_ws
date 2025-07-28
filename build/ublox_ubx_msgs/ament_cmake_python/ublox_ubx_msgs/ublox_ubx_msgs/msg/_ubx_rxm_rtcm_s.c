// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXRxmRTCM.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rtcm__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rtcm__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_rxm_rtcm__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_rxm_rtcm.UBXRxmRTCM", full_classname_dest, 43) == 0);
  }
  ublox_ubx_msgs__msg__UBXRxmRTCM * ros_message = _ros_message;
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
  {  // crc_failed
    PyObject * field = PyObject_GetAttrString(_pymsg, "crc_failed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->crc_failed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // msg_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "msg_used");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->msg_used = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sub_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "sub_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sub_type = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // ref_station
    PyObject * field = PyObject_GetAttrString(_pymsg, "ref_station");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ref_station = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // msg_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "msg_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->msg_type = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__ubx_rxm_rtcm__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXRxmRTCM */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_rxm_rtcm");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXRxmRTCM");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXRxmRTCM * ros_message = (ublox_ubx_msgs__msg__UBXRxmRTCM *)raw_ros_message;
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
  {  // crc_failed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->crc_failed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "crc_failed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // msg_used
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->msg_used);
    {
      int rc = PyObject_SetAttrString(_pymessage, "msg_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sub_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sub_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sub_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ref_station
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ref_station);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ref_station", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // msg_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->msg_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "msg_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
