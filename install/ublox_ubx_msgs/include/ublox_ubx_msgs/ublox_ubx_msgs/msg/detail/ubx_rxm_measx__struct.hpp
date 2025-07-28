// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXRxmMeasx.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__STRUCT_HPP_

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
// Member 'sv_data'
#include "ublox_ubx_msgs/msg/detail/measx_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXRxmMeasx __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXRxmMeasx __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXRxmMeasx_
{
  using Type = UBXRxmMeasx_<ContainerAllocator>;

  explicit UBXRxmMeasx_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->gps_tow = 0ul;
      this->glo_tow = 0ul;
      this->bds_tow = 0ul;
      this->qzss_tow = 0ul;
      this->gps_tow_acc = 0;
      this->glo_tow_acc = 0;
      this->bds_tow_acc = 0;
      this->qzss_tow_acc = 0;
      this->num_sv = 0;
      this->flags = 0;
    }
  }

  explicit UBXRxmMeasx_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->gps_tow = 0ul;
      this->glo_tow = 0ul;
      this->bds_tow = 0ul;
      this->qzss_tow = 0ul;
      this->gps_tow_acc = 0;
      this->glo_tow_acc = 0;
      this->bds_tow_acc = 0;
      this->qzss_tow_acc = 0;
      this->num_sv = 0;
      this->flags = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _version_type =
    uint8_t;
  _version_type version;
  using _gps_tow_type =
    uint32_t;
  _gps_tow_type gps_tow;
  using _glo_tow_type =
    uint32_t;
  _glo_tow_type glo_tow;
  using _bds_tow_type =
    uint32_t;
  _bds_tow_type bds_tow;
  using _qzss_tow_type =
    uint32_t;
  _qzss_tow_type qzss_tow;
  using _gps_tow_acc_type =
    uint16_t;
  _gps_tow_acc_type gps_tow_acc;
  using _glo_tow_acc_type =
    uint16_t;
  _glo_tow_acc_type glo_tow_acc;
  using _bds_tow_acc_type =
    uint16_t;
  _bds_tow_acc_type bds_tow_acc;
  using _qzss_tow_acc_type =
    uint16_t;
  _qzss_tow_acc_type qzss_tow_acc;
  using _num_sv_type =
    uint8_t;
  _num_sv_type num_sv;
  using _flags_type =
    uint8_t;
  _flags_type flags;
  using _sv_data_type =
    std::vector<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>>>;
  _sv_data_type sv_data;

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
  Type & set__gps_tow(
    const uint32_t & _arg)
  {
    this->gps_tow = _arg;
    return *this;
  }
  Type & set__glo_tow(
    const uint32_t & _arg)
  {
    this->glo_tow = _arg;
    return *this;
  }
  Type & set__bds_tow(
    const uint32_t & _arg)
  {
    this->bds_tow = _arg;
    return *this;
  }
  Type & set__qzss_tow(
    const uint32_t & _arg)
  {
    this->qzss_tow = _arg;
    return *this;
  }
  Type & set__gps_tow_acc(
    const uint16_t & _arg)
  {
    this->gps_tow_acc = _arg;
    return *this;
  }
  Type & set__glo_tow_acc(
    const uint16_t & _arg)
  {
    this->glo_tow_acc = _arg;
    return *this;
  }
  Type & set__bds_tow_acc(
    const uint16_t & _arg)
  {
    this->bds_tow_acc = _arg;
    return *this;
  }
  Type & set__qzss_tow_acc(
    const uint16_t & _arg)
  {
    this->qzss_tow_acc = _arg;
    return *this;
  }
  Type & set__num_sv(
    const uint8_t & _arg)
  {
    this->num_sv = _arg;
    return *this;
  }
  Type & set__flags(
    const uint8_t & _arg)
  {
    this->flags = _arg;
    return *this;
  }
  Type & set__sv_data(
    const std::vector<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>>> & _arg)
  {
    this->sv_data = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t TOW_NOT_SET =
    0u;
  static constexpr uint8_t TOW_SET =
    1u;
  static constexpr uint8_t TOW_SET2 =
    2u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXRxmMeasx
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXRxmMeasx
    std::shared_ptr<ublox_ubx_msgs::msg::UBXRxmMeasx_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXRxmMeasx_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->gps_tow != other.gps_tow) {
      return false;
    }
    if (this->glo_tow != other.glo_tow) {
      return false;
    }
    if (this->bds_tow != other.bds_tow) {
      return false;
    }
    if (this->qzss_tow != other.qzss_tow) {
      return false;
    }
    if (this->gps_tow_acc != other.gps_tow_acc) {
      return false;
    }
    if (this->glo_tow_acc != other.glo_tow_acc) {
      return false;
    }
    if (this->bds_tow_acc != other.bds_tow_acc) {
      return false;
    }
    if (this->qzss_tow_acc != other.qzss_tow_acc) {
      return false;
    }
    if (this->num_sv != other.num_sv) {
      return false;
    }
    if (this->flags != other.flags) {
      return false;
    }
    if (this->sv_data != other.sv_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXRxmMeasx_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXRxmMeasx_

// alias to use template instance with default allocator
using UBXRxmMeasx =
  ublox_ubx_msgs::msg::UBXRxmMeasx_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXRxmMeasx_<ContainerAllocator>::TOW_NOT_SET;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXRxmMeasx_<ContainerAllocator>::TOW_SET;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXRxmMeasx_<ContainerAllocator>::TOW_SET2;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_RXM_MEASX__STRUCT_HPP_
