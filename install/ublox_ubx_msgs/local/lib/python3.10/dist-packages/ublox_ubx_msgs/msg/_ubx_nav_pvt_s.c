// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_ubx_msgs:msg/UBXNavPVT.idl
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
#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__struct.h"
#include "ublox_ubx_msgs/msg/detail/ubx_nav_pvt__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__gps_fix__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__gps_fix__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__psmpvt__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__psmpvt__convert_to_py(void * raw_ros_message);
bool ublox_ubx_msgs__msg__carr_soln__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ublox_ubx_msgs__msg__carr_soln__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ublox_ubx_msgs__msg__ubx_nav_pvt__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
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
    assert(strncmp("ublox_ubx_msgs.msg._ubx_nav_pvt.UBXNavPVT", full_classname_dest, 41) == 0);
  }
  ublox_ubx_msgs__msg__UBXNavPVT * ros_message = _ros_message;
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
  {  // year
    PyObject * field = PyObject_GetAttrString(_pymsg, "year");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->year = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // month
    PyObject * field = PyObject_GetAttrString(_pymsg, "month");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->month = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // day
    PyObject * field = PyObject_GetAttrString(_pymsg, "day");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->day = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // hour
    PyObject * field = PyObject_GetAttrString(_pymsg, "hour");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->hour = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // min
    PyObject * field = PyObject_GetAttrString(_pymsg, "min");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->min = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sec
    PyObject * field = PyObject_GetAttrString(_pymsg, "sec");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sec = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // valid_date
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid_date");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid_date = (Py_True == field);
    Py_DECREF(field);
  }
  {  // valid_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid_time");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid_time = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fully_resolved
    PyObject * field = PyObject_GetAttrString(_pymsg, "fully_resolved");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fully_resolved = (Py_True == field);
    Py_DECREF(field);
  }
  {  // valid_mag
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid_mag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid_mag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // t_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->t_acc = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // nano
    PyObject * field = PyObject_GetAttrString(_pymsg, "nano");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->nano = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // gps_fix
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_fix");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__gps_fix__convert_from_py(field, &ros_message->gps_fix)) {
      Py_DECREF(field);
      return false;
    }
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
  {  // psm
    PyObject * field = PyObject_GetAttrString(_pymsg, "psm");
    if (!field) {
      return false;
    }
    if (!ublox_ubx_msgs__msg__psmpvt__convert_from_py(field, &ros_message->psm)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // head_veh_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_veh_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->head_veh_valid = (Py_True == field);
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
  {  // confirmed_avail
    PyObject * field = PyObject_GetAttrString(_pymsg, "confirmed_avail");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->confirmed_avail = (Py_True == field);
    Py_DECREF(field);
  }
  {  // confirmed_date
    PyObject * field = PyObject_GetAttrString(_pymsg, "confirmed_date");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->confirmed_date = (Py_True == field);
    Py_DECREF(field);
  }
  {  // confirmed_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "confirmed_time");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->confirmed_time = (Py_True == field);
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
  {  // lon
    PyObject * field = PyObject_GetAttrString(_pymsg, "lon");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->lon = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // lat
    PyObject * field = PyObject_GetAttrString(_pymsg, "lat");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->lat = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // height
    PyObject * field = PyObject_GetAttrString(_pymsg, "height");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->height = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // hmsl
    PyObject * field = PyObject_GetAttrString(_pymsg, "hmsl");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->hmsl = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // h_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "h_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->h_acc = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // v_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "v_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->v_acc = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // vel_n
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_n");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->vel_n = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // vel_e
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_e");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->vel_e = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // vel_d
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_d");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->vel_d = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // g_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "g_speed");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->g_speed = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // head_mot
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_mot");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->head_mot = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // s_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "s_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->s_acc = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // head_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->head_acc = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // p_dop
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_dop");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->p_dop = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // invalid_llh
    PyObject * field = PyObject_GetAttrString(_pymsg, "invalid_llh");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->invalid_llh = (Py_True == field);
    Py_DECREF(field);
  }
  {  // head_veh
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_veh");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->head_veh = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // mag_dec
    PyObject * field = PyObject_GetAttrString(_pymsg, "mag_dec");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mag_dec = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // mag_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "mag_acc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mag_acc = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_ubx_msgs__msg__ubx_nav_pvt__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UBXNavPVT */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_ubx_msgs.msg._ubx_nav_pvt");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UBXNavPVT");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_ubx_msgs__msg__UBXNavPVT * ros_message = (ublox_ubx_msgs__msg__UBXNavPVT *)raw_ros_message;
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
  {  // year
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->year);
    {
      int rc = PyObject_SetAttrString(_pymessage, "year", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // month
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->month);
    {
      int rc = PyObject_SetAttrString(_pymessage, "month", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // day
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->day);
    {
      int rc = PyObject_SetAttrString(_pymessage, "day", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hour
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->hour);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hour", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sec
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sec);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid_date
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid_date ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid_date", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid_time
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid_time ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fully_resolved
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fully_resolved ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fully_resolved", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid_mag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid_mag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid_mag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->t_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // nano
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->nano);
    {
      int rc = PyObject_SetAttrString(_pymessage, "nano", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gps_fix
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__gps_fix__convert_to_py(&ros_message->gps_fix);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_fix", field);
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
  {  // psm
    PyObject * field = NULL;
    field = ublox_ubx_msgs__msg__psmpvt__convert_to_py(&ros_message->psm);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "psm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_veh_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->head_veh_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_veh_valid", field);
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
  {  // confirmed_avail
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->confirmed_avail ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confirmed_avail", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confirmed_date
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->confirmed_date ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confirmed_date", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confirmed_time
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->confirmed_time ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confirmed_time", field);
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
  {  // lon
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->lon);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lon", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lat
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->lat);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lat", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hmsl
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->hmsl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hmsl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // h_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->h_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "h_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // v_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->v_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "v_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_n
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->vel_n);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_n", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_e
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->vel_e);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_e", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_d
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->vel_d);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // g_speed
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->g_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "g_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_mot
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->head_mot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_mot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // s_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->s_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->head_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // p_dop
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->p_dop);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_dop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // invalid_llh
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->invalid_llh ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "invalid_llh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_veh
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->head_veh);
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_veh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mag_dec
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->mag_dec);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mag_dec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mag_acc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->mag_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mag_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
