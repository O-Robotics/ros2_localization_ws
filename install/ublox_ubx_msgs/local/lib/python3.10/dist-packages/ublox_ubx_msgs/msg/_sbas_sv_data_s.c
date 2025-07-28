// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/SBASSvData.idl
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
#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__struct.h"
#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__sbas_sv_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ublox_ubx_msgs.msg._sbas_sv_data.SBASSvData", full_classname_dest, 43) == 0);
  }
  ublox_ubx_msgs__msg__SBASSvData * ros_message = _ros_message;
  {  // svid
    PyObject * field = PyObject_GetAttrString(_pymsg, "svid");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->svid = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // reserved_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "reserved_1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->reserved_1 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // udre
    PyObject * field = PyObject_GetAttrString(_pymsg, "udre");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->udre = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sv_sys
    PyObject * field = PyObject_GetAttrString(_pymsg, "sv_sys");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sv_sys = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sv_service
    PyObject * field = PyObject_GetAttrString(_pymsg, "sv_service");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sv_service = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // reserved_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "reserved_2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->reserved_2 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // prc
    PyObject * field = PyObject_GetAttrString(_pymsg, "prc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->prc = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // reserved_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "reserved_3");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_UINT8);
      Py_ssize_t size = 2;
      uint8_t * dest = ros_message->reserved_3;
      for (Py_ssize_t i = 0; i < size; ++i) {
        uint8_t tmp = *(npy_uint8 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(uint8_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // ic
    PyObject * field = PyObject_GetAttrString(_pymsg, "ic");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ic = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__sbas_sv_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SBASSvData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._sbas_sv_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SBASSvData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__SBASSvData * ros_message = (ublox_ubx_msgs__msg__SBASSvData *)raw_ros_message;
  {  // svid
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->svid);
    {
      int rc = PyObject_SetAttrString(_pymessage, "svid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reserved_1
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->reserved_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reserved_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // udre
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->udre);
    {
      int rc = PyObject_SetAttrString(_pymessage, "udre", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sv_sys
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sv_sys);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sv_sys", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sv_service
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sv_service);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sv_service", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reserved_2
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->reserved_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reserved_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // prc
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->prc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "prc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reserved_3
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "reserved_3");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_UINT8);
    assert(sizeof(npy_uint8) == sizeof(uint8_t));
    npy_uint8 * dst = (npy_uint8 *)PyArray_GETPTR1(seq_field, 0);
    uint8_t * src = &(ros_message->reserved_3[0]);
    memcpy(dst, src, 2 * sizeof(uint8_t));
    Py_DECREF(field);
  }
  {  // ic
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->ic);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
