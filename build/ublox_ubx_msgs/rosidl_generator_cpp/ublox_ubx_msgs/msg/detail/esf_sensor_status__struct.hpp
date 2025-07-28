// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/ESFSensorStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__ESFSensorStatus __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__ESFSensorStatus __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ESFSensorStatus_
{
  using Type = ESFSensorStatus_<ContainerAllocator>;

  explicit ESFSensorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_data_type = 0;
      this->used = false;
      this->ready = false;
      this->calib_status = 0;
      this->time_status = 0;
      this->freq = 0;
      this->fault_bad_meas = false;
      this->fault_bad_ttag = false;
      this->fault_missing_meas = false;
      this->fault_noisy_meas = false;
    }
  }

  explicit ESFSensorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor_data_type = 0;
      this->used = false;
      this->ready = false;
      this->calib_status = 0;
      this->time_status = 0;
      this->freq = 0;
      this->fault_bad_meas = false;
      this->fault_bad_ttag = false;
      this->fault_missing_meas = false;
      this->fault_noisy_meas = false;
    }
  }

  // field types and members
  using _sensor_data_type_type =
    uint8_t;
  _sensor_data_type_type sensor_data_type;
  using _used_type =
    bool;
  _used_type used;
  using _ready_type =
    bool;
  _ready_type ready;
  using _calib_status_type =
    uint8_t;
  _calib_status_type calib_status;
  using _time_status_type =
    uint8_t;
  _time_status_type time_status;
  using _freq_type =
    uint8_t;
  _freq_type freq;
  using _fault_bad_meas_type =
    bool;
  _fault_bad_meas_type fault_bad_meas;
  using _fault_bad_ttag_type =
    bool;
  _fault_bad_ttag_type fault_bad_ttag;
  using _fault_missing_meas_type =
    bool;
  _fault_missing_meas_type fault_missing_meas;
  using _fault_noisy_meas_type =
    bool;
  _fault_noisy_meas_type fault_noisy_meas;

  // setters for named parameter idiom
  Type & set__sensor_data_type(
    const uint8_t & _arg)
  {
    this->sensor_data_type = _arg;
    return *this;
  }
  Type & set__used(
    const bool & _arg)
  {
    this->used = _arg;
    return *this;
  }
  Type & set__ready(
    const bool & _arg)
  {
    this->ready = _arg;
    return *this;
  }
  Type & set__calib_status(
    const uint8_t & _arg)
  {
    this->calib_status = _arg;
    return *this;
  }
  Type & set__time_status(
    const uint8_t & _arg)
  {
    this->time_status = _arg;
    return *this;
  }
  Type & set__freq(
    const uint8_t & _arg)
  {
    this->freq = _arg;
    return *this;
  }
  Type & set__fault_bad_meas(
    const bool & _arg)
  {
    this->fault_bad_meas = _arg;
    return *this;
  }
  Type & set__fault_bad_ttag(
    const bool & _arg)
  {
    this->fault_bad_ttag = _arg;
    return *this;
  }
  Type & set__fault_missing_meas(
    const bool & _arg)
  {
    this->fault_missing_meas = _arg;
    return *this;
  }
  Type & set__fault_noisy_meas(
    const bool & _arg)
  {
    this->fault_noisy_meas = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CALIB_STATUS_NOT_CALIBRATED =
    0u;
  static constexpr uint8_t CALIB_STATUS_CALIBRATING =
    1u;
  static constexpr uint8_t CALIB_STATUS_CALIBRATED0 =
    2u;
  static constexpr uint8_t CALIB_STATUS_CALIBRATED1 =
    3u;
  static constexpr uint8_t TIME_STATUS_NO_DATA =
    0u;
  static constexpr uint8_t TIME_STATUS_FIRST_BYTE_USED =
    1u;
  static constexpr uint8_t TIME_STATUS_TTAG_PROVIDED =
    3u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__ESFSensorStatus
    std::shared_ptr<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__ESFSensorStatus
    std::shared_ptr<ublox_ubx_msgs::msg::ESFSensorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ESFSensorStatus_ & other) const
  {
    if (this->sensor_data_type != other.sensor_data_type) {
      return false;
    }
    if (this->used != other.used) {
      return false;
    }
    if (this->ready != other.ready) {
      return false;
    }
    if (this->calib_status != other.calib_status) {
      return false;
    }
    if (this->time_status != other.time_status) {
      return false;
    }
    if (this->freq != other.freq) {
      return false;
    }
    if (this->fault_bad_meas != other.fault_bad_meas) {
      return false;
    }
    if (this->fault_bad_ttag != other.fault_bad_ttag) {
      return false;
    }
    if (this->fault_missing_meas != other.fault_missing_meas) {
      return false;
    }
    if (this->fault_noisy_meas != other.fault_noisy_meas) {
      return false;
    }
    return true;
  }
  bool operator!=(const ESFSensorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ESFSensorStatus_

// alias to use template instance with default allocator
using ESFSensorStatus =
  ublox_ubx_msgs::msg::ESFSensorStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ESFSensorStatus_<ContainerAllocator>::CALIB_STATUS_NOT_CALIBRATED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ESFSensorStatus_<ContainerAllocator>::CALIB_STATUS_CALIBRATING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ESFSensorStatus_<ContainerAllocator>::CALIB_STATUS_CALIBRATED0;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ESFSensorStatus_<ContainerAllocator>::CALIB_STATUS_CALIBRATED1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ESFSensorStatus_<ContainerAllocator>::TIME_STATUS_NO_DATA;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ESFSensorStatus_<ContainerAllocator>::TIME_STATUS_FIRST_BYTE_USED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ESFSensorStatus_<ContainerAllocator>::TIME_STATUS_TTAG_PROVIDED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ESF_SENSOR_STATUS__STRUCT_HPP_
