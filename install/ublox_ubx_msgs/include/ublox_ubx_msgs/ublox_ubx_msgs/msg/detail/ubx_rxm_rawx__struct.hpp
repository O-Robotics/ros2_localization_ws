// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmRawx.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__STRUCT_HPP_

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
// Member 'rec_stat'
#include "ublox_ubx_msgs/msg/detail/rec_stat__struct.hpp"
// Member 'rawx_data'
#include "ublox_ubx_msgs/msg/detail/rawx_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXRxmRawx __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXRxmRawx __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXRxmRawx_
{
  using Type = UBXRxmRawx_<ContainerAllocator>;

  explicit UBXRxmRawx_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    rec_stat(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rcv_tow = 0.0;
      this->week = 0;
      this->leap_s = 0;
      this->num_meas = 0;
      this->version = 0;
    }
  }

  explicit UBXRxmRawx_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    rec_stat(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rcv_tow = 0.0;
      this->week = 0;
      this->leap_s = 0;
      this->num_meas = 0;
      this->version = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _rcv_tow_type =
    double;
  _rcv_tow_type rcv_tow;
  using _week_type =
    uint16_t;
  _week_type week;
  using _leap_s_type =
    int8_t;
  _leap_s_type leap_s;
  using _num_meas_type =
    uint8_t;
  _num_meas_type num_meas;
  using _rec_stat_type =
    ublox_ubx_msgs::msg::RecStat_<ContainerAllocator>;
  _rec_stat_type rec_stat;
  using _version_type =
    uint8_t;
  _version_type version;
  using _rawx_data_type =
    std::vector<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>>>;
  _rawx_data_type rawx_data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__rcv_tow(
    const double & _arg)
  {
    this->rcv_tow = _arg;
    return *this;
  }
  Type & set__week(
    const uint16_t & _arg)
  {
    this->week = _arg;
    return *this;
  }
  Type & set__leap_s(
    const int8_t & _arg)
  {
    this->leap_s = _arg;
    return *this;
  }
  Type & set__num_meas(
    const uint8_t & _arg)
  {
    this->num_meas = _arg;
    return *this;
  }
  Type & set__rec_stat(
    const ublox_ubx_msgs::msg::RecStat_<ContainerAllocator> & _arg)
  {
    this->rec_stat = _arg;
    return *this;
  }
  Type & set__version(
    const uint8_t & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__rawx_data(
    const std::vector<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::RawxData_<ContainerAllocator>>> & _arg)
  {
    this->rawx_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXRxmRawx
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXRxmRawx
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmRawx_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXRxmRawx_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->rcv_tow != other.rcv_tow) {
      return false;
    }
    if (this->week != other.week) {
      return false;
    }
    if (this->leap_s != other.leap_s) {
      return false;
    }
    if (this->num_meas != other.num_meas) {
      return false;
    }
    if (this->rec_stat != other.rec_stat) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->rawx_data != other.rawx_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXRxmRawx_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXRxmRawx_

// alias to use template instance with default allocator
using UBXRxmRawx =
  ublox_ubx_msgs::msg::UBXRxmRawx_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RAWX__STRUCT_HPP_
