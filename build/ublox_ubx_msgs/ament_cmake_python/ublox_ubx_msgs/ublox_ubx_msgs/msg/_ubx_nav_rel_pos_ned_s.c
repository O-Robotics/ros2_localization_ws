// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXNavRelPosNED.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_rel_pos_ned__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__carr_soln__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__carr_soln__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_nav_rel_pos_ned__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_nav_rel_pos_ned.UBXNavRelPosNED", full_classname_dest, 55) == 0);
  }
  ublox_ubx_msgs__msg__UBXNavRelPosNED * ros_message = _ros_message;
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
  {  // ref_station_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "ref_station_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ref_station_id = (uint16_t)PyLong_AsUnsignedLong(field);
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
  {  // rel_pos_n
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_n");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rel_pos_n = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rel_pos_e
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_e");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rel_pos_e = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rel_pos_d
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_d");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rel_pos_d = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rel_pos_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_length");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rel_pos_length = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rel_pos_heading
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_heading");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rel_pos_heading = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rel_pos_hp_n
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_hp_n");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rel_pos_hp_n = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rel_pos_hp_e
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_hp_e");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rel_pos_hp_e = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rel_pos_hp_d
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_hp_d");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rel_pos_hp_d = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // rel_pos_hp_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_hp_length");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rel_pos_hp_length = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // acc_n
    PyObject * field = PyObject_GetAttrString(_pymsg, "acc_n");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->acc_n = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // acc_e
    PyObject * field = PyObject_GetAttrString(_pymsg, "acc_e");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->acc_e = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // acc_d
    PyObject * field = PyObject_GetAttrString(_pymsg, "acc_d");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->acc_d = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // acc_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "acc_length");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->acc_length = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // acc_heading
    PyObject * field = PyObject_GetAttrString(_pymsg, "acc_heading");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->acc_heading = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // gnss_fix_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "gnss_fix_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->gnss_fix_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // diff_soln
    PyObject * field = PyObject_GetAttrString(_pymsg, "diff_soln");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->diff_soln = (Py_True == field);
    Py_DECREF(field);
  }
  {  // rel_pos_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->rel_pos_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // carr_soln
    PyObject * field = PyObject_GetAttrString(_pymsg, "carr_soln");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__carr_soln__convert_from_py(field, &ros_message->carr_soln)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // is_moving
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_moving");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_moving = (Py_True == field);
    Py_DECREF(field);
  }
  {  // ref_pos_miss
    PyObject * field = PyObject_GetAttrString(_pymsg, "ref_pos_miss");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->ref_pos_miss = (Py_True == field);
    Py_DECREF(field);
  }
  {  // ref_obs_miss
    PyObject * field = PyObject_GetAttrString(_pymsg, "ref_obs_miss");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->ref_obs_miss = (Py_True == field);
    Py_DECREF(field);
  }
  {  // rel_pos_heading_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_heading_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->rel_pos_heading_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // rel_pos_normalized
    PyObject * field = PyObject_GetAttrString(_pymsg, "rel_pos_normalized");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->rel_pos_normalized = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__ubx_nav_rel_pos_ned__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXNavRelPosNED */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_nav_rel_pos_ned");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXNavRelPosNED");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXNavRelPosNED * ros_message = (ublox_ubx_msgs__msg__UBXNavRelPosNED *)raw_ros_message;
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
  {  // ref_station_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ref_station_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ref_station_id", field);
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
  {  // rel_pos_n
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rel_pos_n);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_n", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_e
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rel_pos_e);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_e", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_d
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rel_pos_d);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_length
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rel_pos_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_heading
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rel_pos_heading);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_heading", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_hp_n
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rel_pos_hp_n);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_hp_n", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_hp_e
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rel_pos_hp_e);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_hp_e", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_hp_d
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rel_pos_hp_d);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_hp_d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_hp_length
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rel_pos_hp_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_hp_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acc_n
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->acc_n);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acc_n", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acc_e
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->acc_e);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acc_e", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acc_d
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->acc_d);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acc_d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acc_length
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->acc_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acc_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acc_heading
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->acc_heading);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acc_heading", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gnss_fix_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->gnss_fix_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gnss_fix_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // diff_soln
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->diff_soln ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "diff_soln", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->rel_pos_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // carr_soln
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__carr_soln__convert_to_py(&ros_message->carr_soln);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "carr_soln", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_moving
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_moving ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_moving", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ref_pos_miss
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->ref_pos_miss ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ref_pos_miss", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ref_obs_miss
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->ref_obs_miss ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ref_obs_miss", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_heading_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->rel_pos_heading_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_heading_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rel_pos_normalized
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->rel_pos_normalized ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rel_pos_normalized", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
