// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/SatFlags.idl
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
#include "ublox_ubx_msgs/msg/detail/sat_flags__struct.h"
#include "ublox_ubx_msgs/msg/detail/sat_flags__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__sat_flags__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
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
    assert(strncmp("ublox_ubx_msgs.msg._sat_flags.SatFlags", full_classname_dest, 38) == 0);
  }
  ublox_ubx_msgs__msg__SatFlags * ros_message = _ros_message;
  {  // quality_ind
    PyObject * field = PyObject_GetAttrString(_pymsg, "quality_ind");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->quality_ind = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sv_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "sv_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->sv_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // health
    PyObject * field = PyObject_GetAttrString(_pymsg, "health");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->health = (uint8_t)PyLong_AsUnsignedLong(field);
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
  {  // smoothed
    PyObject * field = PyObject_GetAttrString(_pymsg, "smoothed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->smoothed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // orbit_source
    PyObject * field = PyObject_GetAttrString(_pymsg, "orbit_source");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->orbit_source = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // eph_avail
    PyObject * field = PyObject_GetAttrString(_pymsg, "eph_avail");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->eph_avail = (Py_True == field);
    Py_DECREF(field);
  }
  {  // alm_avail
    PyObject * field = PyObject_GetAttrString(_pymsg, "alm_avail");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->alm_avail = (Py_True == field);
    Py_DECREF(field);
  }
  {  // ano_avail
    PyObject * field = PyObject_GetAttrString(_pymsg, "ano_avail");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->ano_avail = (Py_True == field);
    Py_DECREF(field);
  }
  {  // aop_avail
    PyObject * field = PyObject_GetAttrString(_pymsg, "aop_avail");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->aop_avail = (Py_True == field);
    Py_DECREF(field);
  }
  {  // sbas_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "sbas_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->sbas_corr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // rtcm_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "rtcm_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->rtcm_corr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // slas_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "slas_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->slas_corr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // spartn_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "spartn_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->spartn_corr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pr_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "pr_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pr_corr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // cr_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "cr_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cr_corr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // do_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "do_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->do_corr_used = (Py_True == field);
    Py_DECREF(field);
  }
  {  // clas_corr_used
    PyObject * field = PyObject_GetAttrString(_pymsg, "clas_corr_used");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->clas_corr_used = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__sat_flags__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SatFlags */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._sat_flags");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SatFlags");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__SatFlags * ros_message = (ublox_ubx_msgs__msg__SatFlags *)raw_ros_message;
  {  // quality_ind
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->quality_ind);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quality_ind", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sv_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->sv_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sv_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // health
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->health);
    {
      int rc = PyObject_SetAttrString(_pymessage, "health", field);
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
  {  // smoothed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->smoothed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "smoothed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // orbit_source
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->orbit_source);
    {
      int rc = PyObject_SetAttrString(_pymessage, "orbit_source", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // eph_avail
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->eph_avail ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "eph_avail", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // alm_avail
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->alm_avail ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "alm_avail", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ano_avail
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->ano_avail ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ano_avail", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aop_avail
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->aop_avail ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aop_avail", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sbas_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->sbas_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sbas_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rtcm_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->rtcm_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rtcm_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // slas_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->slas_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "slas_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // spartn_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->spartn_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "spartn_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pr_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pr_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pr_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cr_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cr_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cr_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // do_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->do_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "do_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // clas_corr_used
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->clas_corr_used ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "clas_corr_used", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
