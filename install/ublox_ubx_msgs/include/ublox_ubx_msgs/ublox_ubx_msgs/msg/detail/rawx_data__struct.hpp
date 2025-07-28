// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/RawxData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'trk_stat'
#include "ublox_ubx_msgs/msg/detail/trk_stat__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__RawxData __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__RawxData __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RawxData_
{
  using Type = RawxData_<ContainerAllocator>;

  explicit RawxData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : trk_stat(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pr_mes = 0.0;
      this->cp_mes = 0.0;
      this->do_mes = 0.0f;
      this->gnss_id = 0;
      this->sv_id = 0;
      this->sig_id = 0;
      this->freq_id = 0;
      this->locktime = 0;
      this->c_no = 0;
      this->pr_stdev = 0;
      this->cp_stdev = 0;
      this->do_stdev = 0;
    }
  }

  explicit RawxData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : trk_stat(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pr_mes = 0.0;
      this->cp_mes = 0.0;
      this->do_mes = 0.0f;
      this->gnss_id = 0;
      this->sv_id = 0;
      this->sig_id = 0;
      this->freq_id = 0;
      this->locktime = 0;
      this->c_no = 0;
      this->pr_stdev = 0;
      this->cp_stdev = 0;
      this->do_stdev = 0;
    }
  }

  // field types and members
  using _pr_mes_type =
    double;
  _pr_mes_type pr_mes;
  using _cp_mes_type =
    double;
  _cp_mes_type cp_mes;
  using _do_mes_type =
    float;
  _do_mes_type do_mes;
  using _gnss_id_type =
    uint8_t;
  _gnss_id_type gnss_id;
  using _sv_id_type =
    uint8_t;
  _sv_id_type sv_id;
  using _sig_id_type =
    uint8_t;
  _sig_id_type sig_id;
  using _freq_id_type =
    uint8_t;
  _freq_id_type freq_id;
  using _locktime_type =
    uint16_t;
  _locktime_type locktime;
  using _c_no_type =
    uint8_t;
  _c_no_type c_no;
  using _pr_stdev_type =
    uint8_t;
  _pr_stdev_type pr_stdev;
  using _cp_stdev_type =
    uint8_t;
  _cp_stdev_type cp_stdev;
  using _do_stdev_type =
    uint8_t;
  _do_stdev_type do_stdev;
  using _trk_stat_type =
    ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator>;
  _trk_stat_type trk_stat;

  // setters for named parameter idiom
  Type & set__pr_mes(
    const double & _arg)
  {
    this->pr_mes = _arg;
    return *this;
  }
  Type & set__cp_mes(
    const double & _arg)
  {
    this->cp_mes = _arg;
    return *this;
  }
  Type & set__do_mes(
    const float & _arg)
  {
    this->do_mes = _arg;
    return *this;
  }
  Type & set__gnss_id(
    const uint8_t & _arg)
  {
    this->gnss_id = _arg;
    return *this;
  }
  Type & set__sv_id(
    const uint8_t & _arg)
  {
    this->sv_id = _arg;
    return *this;
  }
  Type & set__sig_id(
    const uint8_t & _arg)
  {
    this->sig_id = _arg;
    return *this;
  }
  Type & set__freq_id(
    const uint8_t & _arg)
  {
    this->freq_id = _arg;
    return *this;
  }
  Type & set__locktime(
    const uint16_t & _arg)
  {
    this->locktime = _arg;
    return *this;
  }
  Type & set__c_no(
    const uint8_t & _arg)
  {
    this->c_no = _arg;
    return *this;
  }
  Type & set__pr_stdev(
    const uint8_t & _arg)
  {
    this->pr_stdev = _arg;
    return *this;
  }
  Type & set__cp_stdev(
    const uint8_t & _arg)
  {
    this->cp_stdev = _arg;
    return *this;
  }
  Type & set__do_stdev(
    const uint8_t & _arg)
  {
    this->do_stdev = _arg;
    return *this;
  }
  Type & set__trk_stat(
    const ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator> & _arg)
  {
    this->trk_stat = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::RawxData_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::RawxData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__RawxData
    std::shared_ptr<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__RawxData
    std::shared_ptr<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RawxData_ & other) const
  {
    if (this->pr_mes != other.pr_mes) {
      return false;
    }
    if (this->cp_mes != other.cp_mes) {
      return false;
    }
    if (this->do_mes != other.do_mes) {
      return false;
    }
    if (this->gnss_id != other.gnss_id) {
      return false;
    }
    if (this->sv_id != other.sv_id) {
      return false;
    }
    if (this->sig_id != other.sig_id) {
      return false;
    }
    if (this->freq_id != other.freq_id) {
      return false;
    }
    if (this->locktime != other.locktime) {
      return false;
    }
    if (this->c_no != other.c_no) {
      return false;
    }
    if (this->pr_stdev != other.pr_stdev) {
      return false;
    }
    if (this->cp_stdev != other.cp_stdev) {
      return false;
    }
    if (this->do_stdev != other.do_stdev) {
      return false;
    }
    if (this->trk_stat != other.trk_stat) {
      return false;
    }
    return true;
  }
  bool operator!=(const RawxData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RawxData_

// alias to use template instance with default allocator
using RawxData =
  ublox_ubx_msgs::msg::RawxData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__STRUCT_HPP_
