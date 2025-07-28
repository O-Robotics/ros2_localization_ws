// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/SpoofDet.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SPOOF_DET__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SPOOF_DET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__SpoofDet __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__SpoofDet __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SpoofDet_
{
  using Type = SpoofDet_<ContainerAllocator>;

  explicit SpoofDet_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit SpoofDet_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _state_type =
    uint8_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SPOOF_DET_UNKNOWN =
    0u;
  static constexpr uint8_t SPOOF_DET_NO_SPOOFING =
    1u;
  static constexpr uint8_t SPOOF_DET_SPOOFING =
    2u;
  static constexpr uint8_t SPOOF_DET_MULTIPLE_SPOOFING =
    3u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__SpoofDet
    std::shared_ptr<ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__SpoofDet
    std::shared_ptr<ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpoofDet_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpoofDet_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpoofDet_

// alias to use template instance with default allocator
using SpoofDet =
  ublox_ubx_msgs::msg::SpoofDet_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SpoofDet_<ContainerAllocator>::SPOOF_DET_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SpoofDet_<ContainerAllocator>::SPOOF_DET_NO_SPOOFING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SpoofDet_<ContainerAllocator>::SPOOF_DET_SPOOFING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SpoofDet_<ContainerAllocator>::SPOOF_DET_MULTIPLE_SPOOFING;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SPOOF_DET__STRUCT_HPP_
