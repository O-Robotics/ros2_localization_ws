// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXRxmMeasx.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_measx__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_rxm_measx__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "ublox_ubx_msgs/msg/detail/measx_data__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__measx_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__measx_data__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_rxm_measx__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_rxm_measx.UBXRxmMeasx", full_classname_dest, 45) == 0);
  }
  ublox_ubx_msgs__msg__UBXRxmMeasx * ros_message = _ros_message;
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
  {  // gps_tow
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_tow");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->gps_tow = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // glo_tow
    PyObject * field = PyObject_GetAttrString(_pymsg, "glo_tow");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->glo_tow = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // bds_tow
    PyObject * field = PyObject_GetAttrString(_pymsg, "bds_tow");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bds_tow = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // qzss_tow
    PyObject * field = PyObject_GetAttrString(_pymsg, "qzss_tow");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->qzss_tow = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // gps_tow_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_tow_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->gps_tow_acc = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // glo_tow_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "glo_tow_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->glo_tow_acc = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // bds_tow_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "bds_tow_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bds_tow_acc = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // qzss_tow_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "qzss_tow_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->qzss_tow_acc = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_sv
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_sv");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_sv = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // flags
    PyObject * field = PyObject_GetAttrString(_pymsg, "flags");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->flags = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sv_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "sv_data");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'sv_data'");
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
    if (!ublox_ubx_msgs__msg__MeasxData__Sequence__init(&(ros_message->sv_data), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ublox_ubx_msgs__msg__MeasxData__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ublox_ubx_msgs__msg__MeasxData * dest = ros_message->sv_data.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ublox_ubx_msgs__msg__measx_data__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
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
PyObject * ublox_ubx_msgs__msg__ubx_rxm_measx__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXRxmMeasx */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_rxm_measx");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXRxmMeasx");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXRxmMeasx * ros_message = (ublox_ubx_msgs__msg__UBXRxmMeasx *)raw_ros_message;
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
  {  // gps_tow
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->gps_tow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_tow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // glo_tow
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->glo_tow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "glo_tow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bds_tow
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->bds_tow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bds_tow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // qzss_tow
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->qzss_tow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "qzss_tow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gps_tow_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->gps_tow_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_tow_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // glo_tow_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->glo_tow_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "glo_tow_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bds_tow_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->bds_tow_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bds_tow_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // qzss_tow_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->qzss_tow_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "qzss_tow_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_sv
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_sv);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_sv", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // flags
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->flags);
    {
      int rc = PyObject_SetAttrString(_pymessage, "flags", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sv_data
    PyObject * field = NULL;
    size_t size = ros_message->sv_data.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ublox_ubx_msgs__msg__MeasxData * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->sv_data.data[i]);
      PyObject * pyitem = ublox_ubx_msgs__msg__measx_data__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "sv_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
