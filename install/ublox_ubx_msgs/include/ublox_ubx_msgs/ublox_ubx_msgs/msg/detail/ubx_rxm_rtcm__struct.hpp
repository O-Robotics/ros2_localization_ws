// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmRTCM.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXRxmRTCM __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXRxmRTCM __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXRxmRTCM_
{
  using Type = UBXRxmRTCM_<ContainerAllocator>;

  explicit UBXRxmRTCM_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->crc_failed = false;
      this->msg_used = 0;
      this->sub_type = 0;
      this->ref_station = 0;
      this->msg_type = 0;
    }
  }

  explicit UBXRxmRTCM_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->crc_failed = false;
      this->msg_used = 0;
      this->sub_type = 0;
      this->ref_station = 0;
      this->msg_type = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _version_type =
    uint8_t;
  _version_type version;
  using _crc_failed_type =
    bool;
  _crc_failed_type crc_failed;
  using _msg_used_type =
    uint8_t;
  _msg_used_type msg_used;
  using _sub_type_type =
    uint16_t;
  _sub_type_type sub_type;
  using _ref_station_type =
    uint16_t;
  _ref_station_type ref_station;
  using _msg_type_type =
    uint16_t;
  _msg_type_type msg_type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__version(
    const uint8_t & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__crc_failed(
    const bool & _arg)
  {
    this->crc_failed = _arg;
    return *this;
  }
  Type & set__msg_used(
    const uint8_t & _arg)
  {
    this->msg_used = _arg;
    return *this;
  }
  Type & set__sub_type(
    const uint16_t & _arg)
  {
    this->sub_type = _arg;
    return *this;
  }
  Type & set__ref_station(
    const uint16_t & _arg)
  {
    this->ref_station = _arg;
    return *this;
  }
  Type & set__msg_type(
    const uint16_t & _arg)
  {
    this->msg_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXRxmRTCM
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXRxmRTCM
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmRTCM_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXRxmRTCM_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->crc_failed != other.crc_failed) {
      return false;
    }
    if (this->msg_used != other.msg_used) {
      return false;
    }
    if (this->sub_type != other.sub_type) {
      return false;
    }
    if (this->ref_station != other.ref_station) {
      return false;
    }
    if (this->msg_type != other.msg_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXRxmRTCM_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXRxmRTCM_

// alias to use template instance with default allocator
using UBXRxmRTCM =
  ublox_ubx_msgs::msg::UBXRxmRTCM_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_RTCM__STRUCT_HPP_
