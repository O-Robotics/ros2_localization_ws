// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXEsfStatus.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_esf_status__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_esf_status__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__esf_sensor_status__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__esf_sensor_status__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_esf_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_esf_status.UBXEsfStatus", full_classname_dest, 47) == 0);
  }
  ublox_ubx_msgs__msg__UBXEsfStatus * ros_message = _ros_message;
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
  {  // version
    PyObject * field = PyObject_GetAttrString(_pymsg, "version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // wt_init_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "wt_init_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->wt_init_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // mnt_alg_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "mnt_alg_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mnt_alg_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // ins_init_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "ins_init_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ins_init_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // imu_init_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_init_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->imu_init_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // fusion_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "fusion_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fusion_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_sens
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_sens");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_sens = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sensor_statuses
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensor_statuses");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'sensor_statuses'");
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
    if (!ublox_ubx_msgs__msg__ESFSensorStatus__Sequence__init(&(ros_message->sensor_statuses), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ublox_ubx_msgs__msg__ESFSensorStatus__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ublox_ubx_msgs__msg__ESFSensorStatus * dest = ros_message->sensor_statuses.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ublox_ubx_msgs__msg__esf_sensor_status__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
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
PyObject * ublox_ubx_msgs__msg__ubx_esf_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXEsfStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_esf_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXEsfStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXEsfStatus * ros_message = (ublox_ubx_msgs__msg__UBXEsfStatus *)raw_ros_message;
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
  {  // wt_init_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->wt_init_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wt_init_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mnt_alg_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->mnt_alg_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mnt_alg_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ins_init_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ins_init_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ins_init_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_init_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->imu_init_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_init_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fusion_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->fusion_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fusion_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_sens
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_sens);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_sens", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sensor_statuses
    PyObject * field = NULL;
    size_t size = ros_message->sensor_statuses.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ublox_ubx_msgs__msg__ESFSensorStatus * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->sensor_statuses.data[i]);
      PyObject * pyitem = ublox_ubx_msgs__msg__esf_sensor_status__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "sensor_statuses", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
