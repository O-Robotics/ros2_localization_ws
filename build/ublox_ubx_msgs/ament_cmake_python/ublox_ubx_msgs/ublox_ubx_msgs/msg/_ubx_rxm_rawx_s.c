// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXRxmRawx.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_rawx__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "ublox_ubx_msgs/msg/detail/rawx_data__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__rec_stat__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__rec_stat__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__rawx_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__rawx_data__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_rxm_rawx__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_rxm_rawx.UBXRxmRawx", full_classname_dest, 43) == 0);
  }
  ublox_ubx_msgs__msg__UBXRxmRawx * ros_message = _ros_message;
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
  {  // rcv_tow
    PyObject * field = PyObject_GetAttrString(_pymsg, "rcv_tow");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rcv_tow = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // week
    PyObject * field = PyObject_GetAttrString(_pymsg, "week");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->week = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // leap_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "leap_s");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->leap_s = (int8_t)PyLong_AsLong(field);
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
  {  // rec_stat
    PyObject * field = PyObject_GetAttrString(_pymsg, "rec_stat");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__rec_stat__convert_from_py(field, &ros_message->rec_stat)) {
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
  {  // rawx_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "rawx_data");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'rawx_data'");
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
    if (!ublox_ubx_msgs__msg__RawxData__Sequence__init(&(ros_message->rawx_data), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ublox_ubx_msgs__msg__RawxData__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ublox_ubx_msgs__msg__RawxData * dest = ros_message->rawx_data.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ublox_ubx_msgs__msg__rawx_data__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__ubx_rxm_rawx__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXRxmRawx */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_rxm_rawx");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXRxmRawx");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXRxmRawx * ros_message = (ublox_ubx_msgs__msg__UBXRxmRawx *)raw_ros_message;
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
  {  // rcv_tow
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rcv_tow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rcv_tow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // week
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->week);
    {
      int rc = PyObject_SetAttrString(_pymessage, "week", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // leap_s
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->leap_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "leap_s", field);
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
  {  // rec_stat
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__rec_stat__convert_to_py(&ros_message->rec_stat);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "rec_stat", field);
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
  {  // rawx_data
    PyObject * field = NULL;
    size_t size = ros_message->rawx_data.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ublox_ubx_msgs__msg__RawxData * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->rawx_data.data[i]);
      PyObject * pyitem = ublox_ubx_msgs__msg__rawx_data__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "rawx_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
