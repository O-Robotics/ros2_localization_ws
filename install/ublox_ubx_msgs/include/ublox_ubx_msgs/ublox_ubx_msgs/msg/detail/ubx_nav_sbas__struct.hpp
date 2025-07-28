// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavSBAS.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__STRUCT_HPP_

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
// Member 'service'
#include "ublox_ubx_msgs/msg/detail/sbas_service__struct.hpp"
// Member 'status_flags'
#include "ublox_ubx_msgs/msg/detail/sbas_status_flags__struct.hpp"
// Member 'sv_data'
#include "ublox_ubx_msgs/msg/detail/sbas_sv_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavSBAS __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavSBAS __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavSBAS_
{
  using Type = UBXNavSBAS_<ContainerAllocator>;

  explicit UBXNavSBAS_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    service(_init),
    status_flags(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->geo = 0;
      this->mode = 0;
      this->sys = 0;
      this->cnt = 0;
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->reserved_0.begin(), this->reserved_0.end(), 0);
    }
  }

  explicit UBXNavSBAS_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    service(_alloc, _init),
    status_flags(_alloc, _init),
    reserved_0(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->geo = 0;
      this->mode = 0;
      this->sys = 0;
      this->cnt = 0;
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->reserved_0.begin(), this->reserved_0.end(), 0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _geo_type =
    uint8_t;
  _geo_type geo;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _sys_type =
    int8_t;
  _sys_type sys;
  using _service_type =
    ublox_ubx_msgs::msg::SBASService_<ContainerAllocator>;
  _service_type service;
  using _cnt_type =
    uint8_t;
  _cnt_type cnt;
  using _status_flags_type =
    ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator>;
  _status_flags_type status_flags;
  using _reserved_0_type =
    std::array<uint8_t, 2>;
  _reserved_0_type reserved_0;
  using _sv_data_type =
    std::vector<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>>>;
  _sv_data_type sv_data;

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
  Type & set__geo(
    const uint8_t & _arg)
  {
    this->geo = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__sys(
    const int8_t & _arg)
  {
    this->sys = _arg;
    return *this;
  }
  Type & set__service(
    const ublox_ubx_msgs::msg::SBASService_<ContainerAllocator> & _arg)
  {
    this->service = _arg;
    return *this;
  }
  Type & set__cnt(
    const uint8_t & _arg)
  {
    this->cnt = _arg;
    return *this;
  }
  Type & set__status_flags(
    const ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator> & _arg)
  {
    this->status_flags = _arg;
    return *this;
  }
  Type & set__reserved_0(
    const std::array<uint8_t, 2> & _arg)
  {
    this->reserved_0 = _arg;
    return *this;
  }
  Type & set__sv_data(
    const std::vector<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>>> & _arg)
  {
    this->sv_data = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MODE_DISABLED =
    0u;
  static constexpr uint8_t MODE_ENABLED_INTEGRITY =
    1u;
  static constexpr uint8_t MODE_ENABLED_TEST =
    3u;
  static constexpr int8_t SYS_UNKNOWN =
    -1;
  static constexpr int8_t SYS_WAAS =
    0;
  static constexpr int8_t SYS_EGNOS =
    1;
  static constexpr int8_t SYS_MSAS =
    2;
  static constexpr int8_t SYS_GAGAN =
    3;
  static constexpr int8_t SYS_GPS =
    16;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavSBAS
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavSBAS
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavSBAS_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavSBAS_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->geo != other.geo) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->sys != other.sys) {
      return false;
    }
    if (this->service != other.service) {
      return false;
    }
    if (this->cnt != other.cnt) {
      return false;
    }
    if (this->status_flags != other.status_flags) {
      return false;
    }
    if (this->reserved_0 != other.reserved_0) {
      return false;
    }
    if (this->sv_data != other.sv_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavSBAS_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavSBAS_

// alias to use template instance with default allocator
using UBXNavSBAS =
  ublox_ubx_msgs::msg::UBXNavSBAS_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXNavSBAS_<ContainerAllocator>::MODE_DISABLED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXNavSBAS_<ContainerAllocator>::MODE_ENABLED_INTEGRITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXNavSBAS_<ContainerAllocator>::MODE_ENABLED_TEST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t UBXNavSBAS_<ContainerAllocator>::SYS_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t UBXNavSBAS_<ContainerAllocator>::SYS_WAAS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t UBXNavSBAS_<ContainerAllocator>::SYS_EGNOS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t UBXNavSBAS_<ContainerAllocator>::SYS_MSAS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t UBXNavSBAS_<ContainerAllocator>::SYS_GAGAN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t UBXNavSBAS_<ContainerAllocator>::SYS_GPS;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SBAS__STRUCT_HPP_
