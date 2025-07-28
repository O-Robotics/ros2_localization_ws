// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/ESFSensorStatus.idl
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
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__struct.h"
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__esf_sensor_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
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
    assert(strncmp("ublox_ubx_msgs.msg._esf_sensor_status.ESFSensorStatus", full_classname_dest, 53) == 0);
  }
  ublox_ubx_msgs__msg__ESFSensorStatus * ros_message = _ros_message;
  {  // sensor_data_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensor_data_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensor_data_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // used
    PyObject * field = PyObject_GetAttrString(_pymsg, "used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // ready
    PyObject * field = PyObject_GetAttrString(_pymsg, "ready");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->ready = (Py_True == field);
    Py_DECREF(field);
  }
  {  // calib_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "calib_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->calib_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // time_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // freq
    PyObject * field = PyObject_GetAttrString(_pymsg, "freq");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->freq = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // fault_bad_meas
    PyObject * field = PyObject_GetAttrString(_pymsg, "fault_bad_meas");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fault_bad_meas = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fault_bad_ttag
    PyObject * field = PyObject_GetAttrString(_pymsg, "fault_bad_ttag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fault_bad_ttag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fault_missing_meas
    PyObject * field = PyObject_GetAttrString(_pymsg, "fault_missing_meas");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fault_missing_meas = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fault_noisy_meas
    PyObject * field = PyObject_GetAttrString(_pymsg, "fault_noisy_meas");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fault_noisy_meas = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__esf_sensor_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ESFSensorStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._esf_sensor_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ESFSensorStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__ESFSensorStatus * ros_message = (ublox_ubx_msgs__msg__ESFSensorStatus *)raw_ros_message;
  {  // sensor_data_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sensor_data_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensor_data_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ready
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->ready ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ready", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calib_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->calib_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calib_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->time_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // freq
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->freq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "freq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fault_bad_meas
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fault_bad_meas ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fault_bad_meas", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fault_bad_ttag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fault_bad_ttag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fault_bad_ttag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fault_missing_meas
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fault_missing_meas ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fault_missing_meas", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fault_noisy_meas
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fault_noisy_meas ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fault_noisy_meas", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
