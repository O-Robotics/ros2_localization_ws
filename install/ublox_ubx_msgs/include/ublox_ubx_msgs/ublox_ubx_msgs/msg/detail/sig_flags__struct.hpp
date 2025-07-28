// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/SigFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__SigFlags __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__SigFlags __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SigFlags_
{
  using Type = SigFlags_<ContainerAllocator>;

  explicit SigFlags_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->health = 0;
      this->pr_smoothed = false;
      this->pr_used = false;
      this->cr_used = false;
      this->do_used = false;
      this->pr_corr_used = false;
      this->cr_corr_used = false;
      this->do_corr_used = false;
    }
  }

  explicit SigFlags_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->health = 0;
      this->pr_smoothed = false;
      this->pr_used = false;
      this->cr_used = false;
      this->do_used = false;
      this->pr_corr_used = false;
      this->cr_corr_used = false;
      this->do_corr_used = false;
    }
  }

  // field types and members
  using _health_type =
    uint8_t;
  _health_type health;
  using _pr_smoothed_type =
    bool;
  _pr_smoothed_type pr_smoothed;
  using _pr_used_type =
    bool;
  _pr_used_type pr_used;
  using _cr_used_type =
    bool;
  _cr_used_type cr_used;
  using _do_used_type =
    bool;
  _do_used_type do_used;
  using _pr_corr_used_type =
    bool;
  _pr_corr_used_type pr_corr_used;
  using _cr_corr_used_type =
    bool;
  _cr_corr_used_type cr_corr_used;
  using _do_corr_used_type =
    bool;
  _do_corr_used_type do_corr_used;

  // setters for named parameter idiom
  Type & set__health(
    const uint8_t & _arg)
  {
    this->health = _arg;
    return *this;
  }
  Type & set__pr_smoothed(
    const bool & _arg)
  {
    this->pr_smoothed = _arg;
    return *this;
  }
  Type & set__pr_used(
    const bool & _arg)
  {
    this->pr_used = _arg;
    return *this;
  }
  Type & set__cr_used(
    const bool & _arg)
  {
    this->cr_used = _arg;
    return *this;
  }
  Type & set__do_used(
    const bool & _arg)
  {
    this->do_used = _arg;
    return *this;
  }
  Type & set__pr_corr_used(
    const bool & _arg)
  {
    this->pr_corr_used = _arg;
    return *this;
  }
  Type & set__cr_corr_used(
    const bool & _arg)
  {
    this->cr_corr_used = _arg;
    return *this;
  }
  Type & set__do_corr_used(
    const bool & _arg)
  {
    this->do_corr_used = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t HEALTH_UNKNOWN =
    0u;
  static constexpr uint8_t HEALTH_HEALTHY =
    1u;
  static constexpr uint8_t HEALTH_UNHEALTHY =
    2u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__SigFlags
    std::shared_ptr<ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__SigFlags
    std::shared_ptr<ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SigFlags_ & other) const
  {
    if (this->health != other.health) {
      return false;
    }
    if (this->pr_smoothed != other.pr_smoothed) {
      return false;
    }
    if (this->pr_used != other.pr_used) {
      return false;
    }
    if (this->cr_used != other.cr_used) {
      return false;
    }
    if (this->do_used != other.do_used) {
      return false;
    }
    if (this->pr_corr_used != other.pr_corr_used) {
      return false;
    }
    if (this->cr_corr_used != other.cr_corr_used) {
      return false;
    }
    if (this->do_corr_used != other.do_corr_used) {
      return false;
    }
    return true;
  }
  bool operator!=(const SigFlags_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SigFlags_

// alias to use template instance with default allocator
using SigFlags =
  ublox_ubx_msgs::msg::SigFlags_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigFlags_<ContainerAllocator>::HEALTH_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigFlags_<ContainerAllocator>::HEALTH_HEALTHY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigFlags_<ContainerAllocator>::HEALTH_UNHEALTHY;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_FLAGS__STRUCT_HPP_
