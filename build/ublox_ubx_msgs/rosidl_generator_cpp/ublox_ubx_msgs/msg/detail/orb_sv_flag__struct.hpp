// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/OrbSVFlag.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__OrbSVFlag __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__OrbSVFlag __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OrbSVFlag_
{
  using Type = OrbSVFlag_<ContainerAllocator>;

  explicit OrbSVFlag_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->health = 0;
      this->visibility = 0;
    }
  }

  explicit OrbSVFlag_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->health = 0;
      this->visibility = 0;
    }
  }

  // field types and members
  using _health_type =
    uint8_t;
  _health_type health;
  using _visibility_type =
    uint8_t;
  _visibility_type visibility;

  // setters for named parameter idiom
  Type & set__health(
    const uint8_t & _arg)
  {
    this->health = _arg;
    return *this;
  }
  Type & set__visibility(
    const uint8_t & _arg)
  {
    this->visibility = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t HEALTH_UNKNOWN =
    0u;
  static constexpr uint8_t HEALTH_HEALTHY =
    1u;
  static constexpr uint8_t HEALTH_NOT_HEALTHY =
    2u;
  static constexpr uint8_t VISIBILITY_UNKNOWN =
    0u;
  static constexpr uint8_t VISIBILITY_BELOW_HORIZON =
    1u;
  static constexpr uint8_t VISIBILITY_ABOVE_HORIZON =
    2u;
  static constexpr uint8_t VISIBILITY_ABOVE_ELEVATION_MASK =
    3u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__OrbSVFlag
    std::shared_ptr<ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__OrbSVFlag
    std::shared_ptr<ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OrbSVFlag_ & other) const
  {
    if (this->health != other.health) {
      return false;
    }
    if (this->visibility != other.visibility) {
      return false;
    }
    return true;
  }
  bool operator!=(const OrbSVFlag_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OrbSVFlag_

// alias to use template instance with default allocator
using OrbSVFlag =
  ublox_ubx_msgs::msg::OrbSVFlag_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbSVFlag_<ContainerAllocator>::HEALTH_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbSVFlag_<ContainerAllocator>::HEALTH_HEALTHY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbSVFlag_<ContainerAllocator>::HEALTH_NOT_HEALTHY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbSVFlag_<ContainerAllocator>::VISIBILITY_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbSVFlag_<ContainerAllocator>::VISIBILITY_BELOW_HORIZON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbSVFlag_<ContainerAllocator>::VISIBILITY_ABOVE_HORIZON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OrbSVFlag_<ContainerAllocator>::VISIBILITY_ABOVE_ELEVATION_MASK;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_FLAG__STRUCT_HPP_
