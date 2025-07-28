// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPVT.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'gps_fix'
#include "ublox_ubx_msgs/msg/detail/gps_fix__struct.hpp"
// Member 'psm'
#include "ublox_ubx_msgs/msg/detail/psmpvt__struct.hpp"
// Member 'carr_soln'
#include "ublox_ubx_msgs/msg/detail/carr_soln__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavPVT __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavPVT __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavPVT_
{
  using Type = UBXNavPVT_<ContainerAllocator>;

  explicit UBXNavPVT_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    gps_fix(_init),
    psm(_init),
    carr_soln(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->min = 0;
      this->sec = 0;
      this->valid_date = false;
      this->valid_time = false;
      this->fully_resolved = false;
      this->valid_mag = false;
      this->t_acc = 0ul;
      this->nano = 0l;
      this->gnss_fix_ok = false;
      this->diff_soln = false;
      this->head_veh_valid = false;
      this->confirmed_avail = false;
      this->confirmed_date = false;
      this->confirmed_time = false;
      this->num_sv = 0;
      this->lon = 0l;
      this->lat = 0l;
      this->height = 0l;
      this->hmsl = 0l;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
      this->vel_n = 0l;
      this->vel_e = 0l;
      this->vel_d = 0l;
      this->g_speed = 0l;
      this->head_mot = 0l;
      this->s_acc = 0ul;
      this->head_acc = 0ul;
      this->p_dop = 0;
      this->invalid_llh = false;
      this->head_veh = 0l;
      this->mag_dec = 0;
      this->mag_acc = 0;
    }
  }

  explicit UBXNavPVT_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    gps_fix(_alloc, _init),
    psm(_alloc, _init),
    carr_soln(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->min = 0;
      this->sec = 0;
      this->valid_date = false;
      this->valid_time = false;
      this->fully_resolved = false;
      this->valid_mag = false;
      this->t_acc = 0ul;
      this->nano = 0l;
      this->gnss_fix_ok = false;
      this->diff_soln = false;
      this->head_veh_valid = false;
      this->confirmed_avail = false;
      this->confirmed_date = false;
      this->confirmed_time = false;
      this->num_sv = 0;
      this->lon = 0l;
      this->lat = 0l;
      this->height = 0l;
      this->hmsl = 0l;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
      this->vel_n = 0l;
      this->vel_e = 0l;
      this->vel_d = 0l;
      this->g_speed = 0l;
      this->head_mot = 0l;
      this->s_acc = 0ul;
      this->head_acc = 0ul;
      this->p_dop = 0;
      this->invalid_llh = false;
      this->head_veh = 0l;
      this->mag_dec = 0;
      this->mag_acc = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _year_type =
    uint16_t;
  _year_type year;
  using _month_type =
    uint8_t;
  _month_type month;
  using _day_type =
    uint8_t;
  _day_type day;
  using _hour_type =
    uint8_t;
  _hour_type hour;
  using _min_type =
    uint8_t;
  _min_type min;
  using _sec_type =
    uint8_t;
  _sec_type sec;
  using _valid_date_type =
    bool;
  _valid_date_type valid_date;
  using _valid_time_type =
    bool;
  _valid_time_type valid_time;
  using _fully_resolved_type =
    bool;
  _fully_resolved_type fully_resolved;
  using _valid_mag_type =
    bool;
  _valid_mag_type valid_mag;
  using _t_acc_type =
    uint32_t;
  _t_acc_type t_acc;
  using _nano_type =
    int32_t;
  _nano_type nano;
  using _gps_fix_type =
    ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator>;
  _gps_fix_type gps_fix;
  using _gnss_fix_ok_type =
    bool;
  _gnss_fix_ok_type gnss_fix_ok;
  using _diff_soln_type =
    bool;
  _diff_soln_type diff_soln;
  using _psm_type =
    ublox_ubx_msgs::msg::PSMPVT_<ContainerAllocator>;
  _psm_type psm;
  using _head_veh_valid_type =
    bool;
  _head_veh_valid_type head_veh_valid;
  using _carr_soln_type =
    ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator>;
  _carr_soln_type carr_soln;
  using _confirmed_avail_type =
    bool;
  _confirmed_avail_type confirmed_avail;
  using _confirmed_date_type =
    bool;
  _confirmed_date_type confirmed_date;
  using _confirmed_time_type =
    bool;
  _confirmed_time_type confirmed_time;
  using _num_sv_type =
    uint8_t;
  _num_sv_type num_sv;
  using _lon_type =
    int32_t;
  _lon_type lon;
  using _lat_type =
    int32_t;
  _lat_type lat;
  using _height_type =
    int32_t;
  _height_type height;
  using _hmsl_type =
    int32_t;
  _hmsl_type hmsl;
  using _h_acc_type =
    uint32_t;
  _h_acc_type h_acc;
  using _v_acc_type =
    uint32_t;
  _v_acc_type v_acc;
  using _vel_n_type =
    int32_t;
  _vel_n_type vel_n;
  using _vel_e_type =
    int32_t;
  _vel_e_type vel_e;
  using _vel_d_type =
    int32_t;
  _vel_d_type vel_d;
  using _g_speed_type =
    int32_t;
  _g_speed_type g_speed;
  using _head_mot_type =
    int32_t;
  _head_mot_type head_mot;
  using _s_acc_type =
    uint32_t;
  _s_acc_type s_acc;
  using _head_acc_type =
    uint32_t;
  _head_acc_type head_acc;
  using _p_dop_type =
    uint16_t;
  _p_dop_type p_dop;
  using _invalid_llh_type =
    bool;
  _invalid_llh_type invalid_llh;
  using _head_veh_type =
    int32_t;
  _head_veh_type head_veh;
  using _mag_dec_type =
    int16_t;
  _mag_dec_type mag_dec;
  using _mag_acc_type =
    uint16_t;
  _mag_acc_type mag_acc;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__itow(
    const uint32_t & _arg)
  {
    this->itow = _arg;
    return *this;
  }
  Type & set__year(
    const uint16_t & _arg)
  {
    this->year = _arg;
    return *this;
  }
  Type & set__month(
    const uint8_t & _arg)
  {
    this->month = _arg;
    return *this;
  }
  Type & set__day(
    const uint8_t & _arg)
  {
    this->day = _arg;
    return *this;
  }
  Type & set__hour(
    const uint8_t & _arg)
  {
    this->hour = _arg;
    return *this;
  }
  Type & set__min(
    const uint8_t & _arg)
  {
    this->min = _arg;
    return *this;
  }
  Type & set__sec(
    const uint8_t & _arg)
  {
    this->sec = _arg;
    return *this;
  }
  Type & set__valid_date(
    const bool & _arg)
  {
    this->valid_date = _arg;
    return *this;
  }
  Type & set__valid_time(
    const bool & _arg)
  {
    this->valid_time = _arg;
    return *this;
  }
  Type & set__fully_resolved(
    const bool & _arg)
  {
    this->fully_resolved = _arg;
    return *this;
  }
  Type & set__valid_mag(
    const bool & _arg)
  {
    this->valid_mag = _arg;
    return *this;
  }
  Type & set__t_acc(
    const uint32_t & _arg)
  {
    this->t_acc = _arg;
    return *this;
  }
  Type & set__nano(
    const int32_t & _arg)
  {
    this->nano = _arg;
    return *this;
  }
  Type & set__gps_fix(
    const ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator> & _arg)
  {
    this->gps_fix = _arg;
    return *this;
  }
  Type & set__gnss_fix_ok(
    const bool & _arg)
  {
    this->gnss_fix_ok = _arg;
    return *this;
  }
  Type & set__diff_soln(
    const bool & _arg)
  {
    this->diff_soln = _arg;
    return *this;
  }
  Type & set__psm(
    const ublox_ubx_msgs::msg::PSMPVT_<ContainerAllocator> & _arg)
  {
    this->psm = _arg;
    return *this;
  }
  Type & set__head_veh_valid(
    const bool & _arg)
  {
    this->head_veh_valid = _arg;
    return *this;
  }
  Type & set__carr_soln(
    const ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator> & _arg)
  {
    this->carr_soln = _arg;
    return *this;
  }
  Type & set__confirmed_avail(
    const bool & _arg)
  {
    this->confirmed_avail = _arg;
    return *this;
  }
  Type & set__confirmed_date(
    const bool & _arg)
  {
    this->confirmed_date = _arg;
    return *this;
  }
  Type & set__confirmed_time(
    const bool & _arg)
  {
    this->confirmed_time = _arg;
    return *this;
  }
  Type & set__num_sv(
    const uint8_t & _arg)
  {
    this->num_sv = _arg;
    return *this;
  }
  Type & set__lon(
    const int32_t & _arg)
  {
    this->lon = _arg;
    return *this;
  }
  Type & set__lat(
    const int32_t & _arg)
  {
    this->lat = _arg;
    return *this;
  }
  Type & set__height(
    const int32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__hmsl(
    const int32_t & _arg)
  {
    this->hmsl = _arg;
    return *this;
  }
  Type & set__h_acc(
    const uint32_t & _arg)
  {
    this->h_acc = _arg;
    return *this;
  }
  Type & set__v_acc(
    const uint32_t & _arg)
  {
    this->v_acc = _arg;
    return *this;
  }
  Type & set__vel_n(
    const int32_t & _arg)
  {
    this->vel_n = _arg;
    return *this;
  }
  Type & set__vel_e(
    const int32_t & _arg)
  {
    this->vel_e = _arg;
    return *this;
  }
  Type & set__vel_d(
    const int32_t & _arg)
  {
    this->vel_d = _arg;
    return *this;
  }
  Type & set__g_speed(
    const int32_t & _arg)
  {
    this->g_speed = _arg;
    return *this;
  }
  Type & set__head_mot(
    const int32_t & _arg)
  {
    this->head_mot = _arg;
    return *this;
  }
  Type & set__s_acc(
    const uint32_t & _arg)
  {
    this->s_acc = _arg;
    return *this;
  }
  Type & set__head_acc(
    const uint32_t & _arg)
  {
    this->head_acc = _arg;
    return *this;
  }
  Type & set__p_dop(
    const uint16_t & _arg)
  {
    this->p_dop = _arg;
    return *this;
  }
  Type & set__invalid_llh(
    const bool & _arg)
  {
    this->invalid_llh = _arg;
    return *this;
  }
  Type & set__head_veh(
    const int32_t & _arg)
  {
    this->head_veh = _arg;
    return *this;
  }
  Type & set__mag_dec(
    const int16_t & _arg)
  {
    this->mag_dec = _arg;
    return *this;
  }
  Type & set__mag_acc(
    const uint16_t & _arg)
  {
    this->mag_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavPVT
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavPVT
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPVT_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavPVT_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->year != other.year) {
      return false;
    }
    if (this->month != other.month) {
      return false;
    }
    if (this->day != other.day) {
      return false;
    }
    if (this->hour != other.hour) {
      return false;
    }
    if (this->min != other.min) {
      return false;
    }
    if (this->sec != other.sec) {
      return false;
    }
    if (this->valid_date != other.valid_date) {
      return false;
    }
    if (this->valid_time != other.valid_time) {
      return false;
    }
    if (this->fully_resolved != other.fully_resolved) {
      return false;
    }
    if (this->valid_mag != other.valid_mag) {
      return false;
    }
    if (this->t_acc != other.t_acc) {
      return false;
    }
    if (this->nano != other.nano) {
      return false;
    }
    if (this->gps_fix != other.gps_fix) {
      return false;
    }
    if (this->gnss_fix_ok != other.gnss_fix_ok) {
      return false;
    }
    if (this->diff_soln != other.diff_soln) {
      return false;
    }
    if (this->psm != other.psm) {
      return false;
    }
    if (this->head_veh_valid != other.head_veh_valid) {
      return false;
    }
    if (this->carr_soln != other.carr_soln) {
      return false;
    }
    if (this->confirmed_avail != other.confirmed_avail) {
      return false;
    }
    if (this->confirmed_date != other.confirmed_date) {
      return false;
    }
    if (this->confirmed_time != other.confirmed_time) {
      return false;
    }
    if (this->num_sv != other.num_sv) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    if (this->lat != other.lat) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->hmsl != other.hmsl) {
      return false;
    }
    if (this->h_acc != other.h_acc) {
      return false;
    }
    if (this->v_acc != other.v_acc) {
      return false;
    }
    if (this->vel_n != other.vel_n) {
      return false;
    }
    if (this->vel_e != other.vel_e) {
      return false;
    }
    if (this->vel_d != other.vel_d) {
      return false;
    }
    if (this->g_speed != other.g_speed) {
      return false;
    }
    if (this->head_mot != other.head_mot) {
      return false;
    }
    if (this->s_acc != other.s_acc) {
      return false;
    }
    if (this->head_acc != other.head_acc) {
      return false;
    }
    if (this->p_dop != other.p_dop) {
      return false;
    }
    if (this->invalid_llh != other.invalid_llh) {
      return false;
    }
    if (this->head_veh != other.head_veh) {
      return false;
    }
    if (this->mag_dec != other.mag_dec) {
      return false;
    }
    if (this->mag_acc != other.mag_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavPVT_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavPVT_

// alias to use template instance with default allocator
using UBXNavPVT =
  ublox_ubx_msgs::msg::UBXNavPVT_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_PVT__STRUCT_HPP_
