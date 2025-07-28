// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/SBASStatusFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__SBASStatusFlags __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__SBASStatusFlags __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SBASStatusFlags_
{
  using Type = SBASStatusFlags_<ContainerAllocator>;

  explicit SBASStatusFlags_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->integrity_used = 0;
    }
  }

  explicit SBASStatusFlags_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->integrity_used = 0;
    }
  }

  // field types and members
  using _integrity_used_type =
    uint8_t;
  _integrity_used_type integrity_used;

  // setters for named parameter idiom
  Type & set__integrity_used(
    const uint8_t & _arg)
  {
    this->integrity_used = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t INTEGRITY_UNKNOWN =
    0u;
  static constexpr uint8_t INTEGRITY_NOT_AVAILABLE =
    1u;
  static constexpr uint8_t INTEGRITY_USED =
    2u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__SBASStatusFlags
    std::shared_ptr<ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__SBASStatusFlags
    std::shared_ptr<ublox_ubx_msgs::msg::SBASStatusFlags_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SBASStatusFlags_ & other) const
  {
    if (this->integrity_used != other.integrity_used) {
      return false;
    }
    return true;
  }
  bool operator!=(const SBASStatusFlags_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SBASStatusFlags_

// alias to use template instance with default allocator
using SBASStatusFlags =
  ublox_ubx_msgs::msg::SBASStatusFlags_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SBASStatusFlags_<ContainerAllocator>::INTEGRITY_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SBASStatusFlags_<ContainerAllocator>::INTEGRITY_NOT_AVAILABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SBASStatusFlags_<ContainerAllocator>::INTEGRITY_USED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_STATUS_FLAGS__STRUCT_HPP_
