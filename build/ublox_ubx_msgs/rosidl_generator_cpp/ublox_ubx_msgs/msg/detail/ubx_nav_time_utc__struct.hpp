// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavTimeUTC.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__STRUCT_HPP_

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
// Member 'utc_std'
#include "ublox_ubx_msgs/msg/detail/utc_std__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavTimeUTC __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavTimeUTC __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavTimeUTC_
{
  using Type = UBXNavTimeUTC_<ContainerAllocator>;

  explicit UBXNavTimeUTC_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    utc_std(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->t_acc = 0ul;
      this->nano = 0l;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->min = 0;
      this->sec = 0;
      this->valid_tow = false;
      this->valid_wkn = false;
      this->valid_utc = false;
    }
  }

  explicit UBXNavTimeUTC_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    utc_std(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->t_acc = 0ul;
      this->nano = 0l;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->min = 0;
      this->sec = 0;
      this->valid_tow = false;
      this->valid_wkn = false;
      this->valid_utc = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _t_acc_type =
    uint32_t;
  _t_acc_type t_acc;
  using _nano_type =
    int32_t;
  _nano_type nano;
  using _year_type =
    int16_t;
  _year_type year;
  using _month_type =
    int8_t;
  _month_type month;
  using _day_type =
    int8_t;
  _day_type day;
  using _hour_type =
    int8_t;
  _hour_type hour;
  using _min_type =
    int8_t;
  _min_type min;
  using _sec_type =
    int8_t;
  _sec_type sec;
  using _valid_tow_type =
    bool;
  _valid_tow_type valid_tow;
  using _valid_wkn_type =
    bool;
  _valid_wkn_type valid_wkn;
  using _valid_utc_type =
    bool;
  _valid_utc_type valid_utc;
  using _utc_std_type =
    ublox_ubx_msgs::msg::UtcStd_<ContainerAllocator>;
  _utc_std_type utc_std;

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
  Type & set__year(
    const int16_t & _arg)
  {
    this->year = _arg;
    return *this;
  }
  Type & set__month(
    const int8_t & _arg)
  {
    this->month = _arg;
    return *this;
  }
  Type & set__day(
    const int8_t & _arg)
  {
    this->day = _arg;
    return *this;
  }
  Type & set__hour(
    const int8_t & _arg)
  {
    this->hour = _arg;
    return *this;
  }
  Type & set__min(
    const int8_t & _arg)
  {
    this->min = _arg;
    return *this;
  }
  Type & set__sec(
    const int8_t & _arg)
  {
    this->sec = _arg;
    return *this;
  }
  Type & set__valid_tow(
    const bool & _arg)
  {
    this->valid_tow = _arg;
    return *this;
  }
  Type & set__valid_wkn(
    const bool & _arg)
  {
    this->valid_wkn = _arg;
    return *this;
  }
  Type & set__valid_utc(
    const bool & _arg)
  {
    this->valid_utc = _arg;
    return *this;
  }
  Type & set__utc_std(
    const ublox_ubx_msgs::msg::UtcStd_<ContainerAllocator> & _arg)
  {
    this->utc_std = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavTimeUTC
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavTimeUTC
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavTimeUTC_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavTimeUTC_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->t_acc != other.t_acc) {
      return false;
    }
    if (this->nano != other.nano) {
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
    if (this->valid_tow != other.valid_tow) {
      return false;
    }
    if (this->valid_wkn != other.valid_wkn) {
      return false;
    }
    if (this->valid_utc != other.valid_utc) {
      return false;
    }
    if (this->utc_std != other.utc_std) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavTimeUTC_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavTimeUTC_

// alias to use template instance with default allocator
using UBXNavTimeUTC =
  ublox_ubx_msgs::msg::UBXNavTimeUTC_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_TIME_UTC__STRUCT_HPP_
