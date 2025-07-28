// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXEsfStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__STRUCT_HPP_

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
// Member 'sensor_statuses'
#include "ublox_ubx_msgs/msg/detail/esf_sensor_status__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXEsfStatus __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXEsfStatus __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXEsfStatus_
{
  using Type = UBXEsfStatus_<ContainerAllocator>;

  explicit UBXEsfStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->version = 0;
      this->wt_init_status = 0;
      this->mnt_alg_status = 0;
      this->ins_init_status = 0;
      this->imu_init_status = 0;
      this->fusion_mode = 0;
      this->num_sens = 0;
    }
  }

  explicit UBXEsfStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->version = 0;
      this->wt_init_status = 0;
      this->mnt_alg_status = 0;
      this->ins_init_status = 0;
      this->imu_init_status = 0;
      this->fusion_mode = 0;
      this->num_sens = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _version_type =
    uint8_t;
  _version_type version;
  using _wt_init_status_type =
    uint8_t;
  _wt_init_status_type wt_init_status;
  using _mnt_alg_status_type =
    uint8_t;
  _mnt_alg_status_type mnt_alg_status;
  using _ins_init_status_type =
    uint8_t;
  _ins_init_status_type ins_init_status;
  using _imu_init_status_type =
    uint8_t;
  _imu_init_status_type imu_init_status;
  using _fusion_mode_type =
    uint8_t;
  _fusion_mode_type fusion_mode;
  using _num_sens_type =
    uint8_t;
  _num_sens_type num_sens;
  using _sensor_statuses_type =
    std::vector<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>>>;
  _sensor_statuses_type sensor_statuses;

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
  Type & set__version(
    const uint8_t & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__wt_init_status(
    const uint8_t & _arg)
  {
    this->wt_init_status = _arg;
    return *this;
  }
  Type & set__mnt_alg_status(
    const uint8_t & _arg)
  {
    this->mnt_alg_status = _arg;
    return *this;
  }
  Type & set__ins_init_status(
    const uint8_t & _arg)
  {
    this->ins_init_status = _arg;
    return *this;
  }
  Type & set__imu_init_status(
    const uint8_t & _arg)
  {
    this->imu_init_status = _arg;
    return *this;
  }
  Type & set__fusion_mode(
    const uint8_t & _arg)
  {
    this->fusion_mode = _arg;
    return *this;
  }
  Type & set__num_sens(
    const uint8_t & _arg)
  {
    this->num_sens = _arg;
    return *this;
  }
  Type & set__sensor_statuses(
    const std::vector<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>>> & _arg)
  {
    this->sensor_statuses = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t WT_INIT_STATUS_OFF =
    0u;
  static constexpr uint8_t WT_INIT_STATUS_INITIALIZING =
    1u;
  static constexpr uint8_t WT_INIT_STATUS_INITIALIZED =
    2u;
  static constexpr uint8_t MBT_ALG_STATUS_OFF =
    0u;
  static constexpr uint8_t MBT_ALG_STATUS_INITIALIZING =
    1u;
  static constexpr uint8_t MBT_ALG_STATUS_INITIALIZED0 =
    2u;
  static constexpr uint8_t MBT_ALG_STATUS_INITIALIZED1 =
    3u;
  static constexpr uint8_t INS_INIT_STATUS_OFF =
    0u;
  static constexpr uint8_t INS_INIT_STATUS_INITIALIZING =
    1u;
  static constexpr uint8_t INS_INIT_STATUS_INITIALIZED =
    2u;
  static constexpr uint8_t IMU_INIT_STATUS_OFF =
    0u;
  static constexpr uint8_t IMU_INIT_STATUS_INITIALIZING =
    1u;
  static constexpr uint8_t IMU_INIT_STATUS_INITIALIZED =
    2u;
  static constexpr uint8_t FUSION_MODE_INITIALIZATION =
    0u;
  static constexpr uint8_t FUSION_MODE_WORKING =
    1u;
  static constexpr uint8_t FUSION_MODE_SUSPENDED =
    2u;
  static constexpr uint8_t FUSION_MODE_DISABLED =
    3u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXEsfStatus
    std::shared_ptr<ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXEsfStatus
    std::shared_ptr<ublox_ubx_msgs::msg::UBXEsfStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXEsfStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->wt_init_status != other.wt_init_status) {
      return false;
    }
    if (this->mnt_alg_status != other.mnt_alg_status) {
      return false;
    }
    if (this->ins_init_status != other.ins_init_status) {
      return false;
    }
    if (this->imu_init_status != other.imu_init_status) {
      return false;
    }
    if (this->fusion_mode != other.fusion_mode) {
      return false;
    }
    if (this->num_sens != other.num_sens) {
      return false;
    }
    if (this->sensor_statuses != other.sensor_statuses) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXEsfStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXEsfStatus_

// alias to use template instance with default allocator
using UBXEsfStatus =
  ublox_ubx_msgs::msg::UBXEsfStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::WT_INIT_STATUS_OFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::WT_INIT_STATUS_INITIALIZING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::WT_INIT_STATUS_INITIALIZED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::MBT_ALG_STATUS_OFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::MBT_ALG_STATUS_INITIALIZING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::MBT_ALG_STATUS_INITIALIZED0;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::MBT_ALG_STATUS_INITIALIZED1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::INS_INIT_STATUS_OFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::INS_INIT_STATUS_INITIALIZING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::INS_INIT_STATUS_INITIALIZED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::IMU_INIT_STATUS_OFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::IMU_INIT_STATUS_INITIALIZING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::IMU_INIT_STATUS_INITIALIZED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::FUSION_MODE_INITIALIZATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::FUSION_MODE_WORKING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::FUSION_MODE_SUSPENDED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXEsfStatus_<ContainerAllocator>::FUSION_MODE_DISABLED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_STATUS__STRUCT_HPP_
