// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXSecSig.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXSecSig __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXSecSig __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXSecSig_
{
  using Type = UBXSecSig_<ContainerAllocator>;

  explicit UBXSecSig_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->jam_det_enabled = 0;
      this->jamming_state = 0;
      this->spf_det_enabled = 0;
      this->spoofing_state = 0;
    }
  }

  explicit UBXSecSig_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->jam_det_enabled = 0;
      this->jamming_state = 0;
      this->spf_det_enabled = 0;
      this->spoofing_state = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _version_type =
    uint8_t;
  _version_type version;
  using _jam_det_enabled_type =
    uint8_t;
  _jam_det_enabled_type jam_det_enabled;
  using _jamming_state_type =
    uint8_t;
  _jamming_state_type jamming_state;
  using _spf_det_enabled_type =
    uint8_t;
  _spf_det_enabled_type spf_det_enabled;
  using _spoofing_state_type =
    uint8_t;
  _spoofing_state_type spoofing_state;

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
  Type & set__jam_det_enabled(
    const uint8_t & _arg)
  {
    this->jam_det_enabled = _arg;
    return *this;
  }
  Type & set__jamming_state(
    const uint8_t & _arg)
  {
    this->jamming_state = _arg;
    return *this;
  }
  Type & set__spf_det_enabled(
    const uint8_t & _arg)
  {
    this->spf_det_enabled = _arg;
    return *this;
  }
  Type & set__spoofing_state(
    const uint8_t & _arg)
  {
    this->spoofing_state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t JAM_UNKNOWN =
    0u;
  static constexpr uint8_t JAM_NO_JAMMING =
    1u;
  static constexpr uint8_t JAM_WARNING =
    2u;
  static constexpr uint8_t JAM_CRITICAL =
    3u;
  static constexpr uint8_t SPF_UNKNOWN =
    0u;
  static constexpr uint8_t SPF_NO_SPOOFING =
    1u;
  static constexpr uint8_t SPF_SPOOFING_INDICATED =
    2u;
  static constexpr uint8_t SPF_SPOOFING_AFFIRMED =
    3u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXSecSig
    std::shared_ptr<ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXSecSig
    std::shared_ptr<ublox_ubx_msgs::msg::UBXSecSig_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXSecSig_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->jam_det_enabled != other.jam_det_enabled) {
      return false;
    }
    if (this->jamming_state != other.jamming_state) {
      return false;
    }
    if (this->spf_det_enabled != other.spf_det_enabled) {
      return false;
    }
    if (this->spoofing_state != other.spoofing_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXSecSig_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXSecSig_

// alias to use template instance with default allocator
using UBXSecSig =
  ublox_ubx_msgs::msg::UBXSecSig_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXSecSig_<ContainerAllocator>::JAM_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXSecSig_<ContainerAllocator>::JAM_NO_JAMMING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXSecSig_<ContainerAllocator>::JAM_WARNING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXSecSig_<ContainerAllocator>::JAM_CRITICAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXSecSig_<ContainerAllocator>::SPF_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXSecSig_<ContainerAllocator>::SPF_NO_SPOOFING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXSecSig_<ContainerAllocator>::SPF_SPOOFING_INDICATED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t UBXSecSig_<ContainerAllocator>::SPF_SPOOFING_AFFIRMED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG__STRUCT_HPP_
