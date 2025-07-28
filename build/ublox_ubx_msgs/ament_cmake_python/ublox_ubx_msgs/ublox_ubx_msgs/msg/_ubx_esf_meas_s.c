// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXEsfMeas.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_esf_meas__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_esf_meas__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "ublox_ubx_msgs/msg/detail/esf_meas_data_item__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__esf_meas_data_item__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__esf_meas_data_item__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_esf_meas__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_esf_meas.UBXEsfMeas", full_classname_dest, 43) == 0);
  }
  ublox_ubx_msgs__msg__UBXEsfMeas * ros_message = _ros_message;
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
  {  // time_tag
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_tag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_tag = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // time_mark_sent
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_mark_sent");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_mark_sent = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // time_mark_edge
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_mark_edge");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->time_mark_edge = (Py_True == field);
    Py_DECREF(field);
  }
  {  // calib_ttag_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "calib_ttag_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->calib_ttag_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // num_meas
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_meas");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_meas = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // data
    PyObject * field = PyObject_GetAttrString(_pymsg, "data");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'data'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence__init(&(ros_message->data), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ublox_ubx_msgs__msg__ESFMeasDataItem__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ublox_ubx_msgs__msg__ESFMeasDataItem * dest = ros_message->data.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ublox_ubx_msgs__msg__esf_meas_data_item__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // calib_ttag
    PyObject * field = PyObject_GetAttrString(_pymsg, "calib_ttag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->calib_ttag = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__ubx_esf_meas__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXEsfMeas */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_esf_meas");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXEsfMeas");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXEsfMeas * ros_message = (ublox_ubx_msgs__msg__UBXEsfMeas *)raw_ros_message;
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
  {  // time_tag
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->time_tag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_tag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_mark_sent
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->time_mark_sent);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_mark_sent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_mark_edge
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->time_mark_edge ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_mark_edge", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calib_ttag_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->calib_ttag_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calib_ttag_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_meas
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_meas);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_meas", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // data
    PyObject * field = NULL;
    size_t size = ros_message->data.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ublox_ubx_msgs__msg__ESFMeasDataItem * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->data.data[i]);
      PyObject * pyitem = ublox_ubx_msgs__msg__esf_meas_data_item__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calib_ttag
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->calib_ttag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calib_ttag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
