// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/MeasxData.idl
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
#include "ublox_ubx_msgs/msg/detail/measx_data__struct.h"
#include "ublox_ubx_msgs/msg/detail/measx_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__measx_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
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
    assert(strncmp("ublox_ubx_msgs.msg._measx_data.MeasxData", full_classname_dest, 40) == 0);
  }
  ublox_ubx_msgs__msg__MeasxData * ros_message = _ros_message;
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
  {  // c_no
    PyObject * field = PyObject_GetAttrString(_pymsg, "c_no");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->c_no = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // mpath_indic
    PyObject * field = PyObject_GetAttrString(_pymsg, "mpath_indic");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mpath_indic = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // doppler_ms
    PyObject * field = PyObject_GetAttrString(_pymsg, "doppler_ms");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->doppler_ms = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // doppler_hz
    PyObject * field = PyObject_GetAttrString(_pymsg, "doppler_hz");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->doppler_hz = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // whole_chips
    PyObject * field = PyObject_GetAttrString(_pymsg, "whole_chips");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->whole_chips = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // frac_chips
    PyObject * field = PyObject_GetAttrString(_pymsg, "frac_chips");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->frac_chips = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // code_phase
    PyObject * field = PyObject_GetAttrString(_pymsg, "code_phase");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->code_phase = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // int_code_phase
    PyObject * field = PyObject_GetAttrString(_pymsg, "int_code_phase");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->int_code_phase = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // pseu_range_rms_err
    PyObject * field = PyObject_GetAttrString(_pymsg, "pseu_range_rms_err");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pseu_range_rms_err = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__measx_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MeasxData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._measx_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MeasxData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__MeasxData * ros_message = (ublox_ubx_msgs__msg__MeasxData *)raw_ros_message;
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
  {  // c_no
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->c_no);
    {
      int rc = PyObject_SetAttrString(_pymessage, "c_no", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mpath_indic
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->mpath_indic);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mpath_indic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // doppler_ms
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->doppler_ms);
    {
      int rc = PyObject_SetAttrString(_pymessage, "doppler_ms", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // doppler_hz
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->doppler_hz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "doppler_hz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // whole_chips
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->whole_chips);
    {
      int rc = PyObject_SetAttrString(_pymessage, "whole_chips", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // frac_chips
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->frac_chips);
    {
      int rc = PyObject_SetAttrString(_pymessage, "frac_chips", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // code_phase
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->code_phase);
    {
      int rc = PyObject_SetAttrString(_pymessage, "code_phase", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // int_code_phase
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->int_code_phase);
    {
      int rc = PyObject_SetAttrString(_pymessage, "int_code_phase", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pseu_range_rms_err
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->pseu_range_rms_err);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pseu_range_rms_err", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
