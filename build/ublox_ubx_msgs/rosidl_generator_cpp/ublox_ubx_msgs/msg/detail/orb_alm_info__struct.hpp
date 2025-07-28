// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/OrbAlmInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_ALM_INFO__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_ALM_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__OrbAlmInfo __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__OrbAlmInfo __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OrbAlmInfo_
{
  using Type = OrbAlmInfo_<ContainerAllocator>;

  explicit OrbAlmInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->alm_usability = 0;
      this->alm_source = 0;
    }
  }

  explicit OrbAlmInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->alm_usability = 0;
      this->alm_source = 0;
    }
  }

  // field types and members
  using _alm_usability_type =
    uint8_t;
  _alm_usability_type alm_usability;
  using _alm_source_type =
    uint8_t;
  _alm_source_type alm_source;

  // setters for named parameter idiom
  Type & set__alm_usability(
    const uint8_t & _arg)
  {
    this->alm_usability = _arg;
    return *this;
  }
  Type & set__alm_source(
    const uint8_t & _arg)
  {
    this->alm_source = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ALM_USABILITY_UNKNOWN =
    31u;
  static constexpr uint8_t ALM_USABILITY_OVER_30_DAYS =
    30u;
  static constexpr uint8_t ALM_USABILITY_EXPIRED =
    0u;
  static constexpr uint8_t ALM_SOURCE_NOT_AVAILABLE =
    0u;
  static constexpr uint8_t ALM_SOURCE_GNSS_TRANSMISSION =
    1u;
  static constexpr uint8_t ALM_SOURCE_EXTERNAL_AIDING =
    2u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__OrbAlmInfo
    std::shared_ptr<ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__OrbAlmInfo
    std::shared_ptr<ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OrbAlmInfo_ & other) const
  {
    if (this->alm_usability != other.alm_usability) {
      return false;
    }
    if (this->alm_source != other.alm_source) {
      return false;
    }
    return true;
  }
  bool operator!=(const OrbAlmInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OrbAlmInfo_

// alias to use template instance with default allocator
using OrbAlmInfo =
  ublox_ubx_msgs::msg::OrbAlmInfo_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbAlmInfo_<ContainerAllocator>::ALM_USABILITY_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbAlmInfo_<ContainerAllocator>::ALM_USABILITY_OVER_30_DAYS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbAlmInfo_<ContainerAllocator>::ALM_USABILITY_EXPIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbAlmInfo_<ContainerAllocator>::ALM_SOURCE_NOT_AVAILABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbAlmInfo_<ContainerAllocator>::ALM_SOURCE_GNSS_TRANSMISSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbAlmInfo_<ContainerAllocator>::ALM_SOURCE_EXTERNAL_AIDING;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_ALM_INFO__STRUCT_HPP_
