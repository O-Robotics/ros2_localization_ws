// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/OrbEphInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__OrbEphInfo __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__OrbEphInfo __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OrbEphInfo_
{
  using Type = OrbEphInfo_<ContainerAllocator>;

  explicit OrbEphInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->eph_usability = 0;
      this->eph_source = 0;
    }
  }

  explicit OrbEphInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->eph_usability = 0;
      this->eph_source = 0;
    }
  }

  // field types and members
  using _eph_usability_type =
    uint8_t;
  _eph_usability_type eph_usability;
  using _eph_source_type =
    uint8_t;
  _eph_source_type eph_source;

  // setters for named parameter idiom
  Type & set__eph_usability(
    const uint8_t & _arg)
  {
    this->eph_usability = _arg;
    return *this;
  }
  Type & set__eph_source(
    const uint8_t & _arg)
  {
    this->eph_source = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t EPH_USABILITY_UNKNOWN =
    31u;
  static constexpr uint8_t EPH_USABILITY_OVER_450_MIN =
    30u;
  static constexpr uint8_t EPH_USABILITY_EXPIRED =
    0u;
  static constexpr uint8_t EPH_SOURCE_NOT_AVAILABLE =
    0u;
  static constexpr uint8_t EPH_SOURCE_GNSS_TRANSMISSION =
    1u;
  static constexpr uint8_t EPH_SOURCE_EXTERNAL_AIDING =
    2u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__OrbEphInfo
    std::shared_ptr<ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__OrbEphInfo
    std::shared_ptr<ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OrbEphInfo_ & other) const
  {
    if (this->eph_usability != other.eph_usability) {
      return false;
    }
    if (this->eph_source != other.eph_source) {
      return false;
    }
    return true;
  }
  bool operator!=(const OrbEphInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OrbEphInfo_

// alias to use template instance with default allocator
using OrbEphInfo =
  ublox_ubx_msgs::msg::OrbEphInfo_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbEphInfo_<ContainerAllocator>::EPH_USABILITY_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbEphInfo_<ContainerAllocator>::EPH_USABILITY_OVER_450_MIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbEphInfo_<ContainerAllocator>::EPH_USABILITY_EXPIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbEphInfo_<ContainerAllocator>::EPH_SOURCE_NOT_AVAILABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbEphInfo_<ContainerAllocator>::EPH_SOURCE_GNSS_TRANSMISSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbEphInfo_<ContainerAllocator>::EPH_SOURCE_EXTERNAL_AIDING;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_EPH_INFO__STRUCT_HPP_
