// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/OtherOrbInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__OTHER_ORB_INFO__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__OTHER_ORB_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__OtherOrbInfo __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__OtherOrbInfo __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OtherOrbInfo_
{
  using Type = OtherOrbInfo_<ContainerAllocator>;

  explicit OtherOrbInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ano_aop_usability = 0;
      this->orb_type = 0;
    }
  }

  explicit OtherOrbInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ano_aop_usability = 0;
      this->orb_type = 0;
    }
  }

  // field types and members
  using _ano_aop_usability_type =
    uint8_t;
  _ano_aop_usability_type ano_aop_usability;
  using _orb_type_type =
    uint8_t;
  _orb_type_type orb_type;

  // setters for named parameter idiom
  Type & set__ano_aop_usability(
    const uint8_t & _arg)
  {
    this->ano_aop_usability = _arg;
    return *this;
  }
  Type & set__orb_type(
    const uint8_t & _arg)
  {
    this->orb_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ANO_AOP_USABILITY_UNKNOWN =
    31u;
  static constexpr uint8_t ANO_AOP_USABILITY_OVER_30_DAYS =
    30u;
  static constexpr uint8_t ANO_AOP_USABILITY_EXPIRED =
    0u;
  static constexpr uint8_t TYPE_NO_ORBIT_DATA =
    0u;
  static constexpr uint8_t TYPE_ASSISTNOW_OFFLINE =
    1u;
  static constexpr uint8_t TYPE_ASSISTNOW_AUTONOMOUS =
    2u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__OtherOrbInfo
    std::shared_ptr<ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__OtherOrbInfo
    std::shared_ptr<ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OtherOrbInfo_ & other) const
  {
    if (this->ano_aop_usability != other.ano_aop_usability) {
      return false;
    }
    if (this->orb_type != other.orb_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const OtherOrbInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OtherOrbInfo_

// alias to use template instance with default allocator
using OtherOrbInfo =
  ublox_ubx_msgs::msg::OtherOrbInfo_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OtherOrbInfo_<ContainerAllocator>::ANO_AOP_USABILITY_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OtherOrbInfo_<ContainerAllocator>::ANO_AOP_USABILITY_OVER_30_DAYS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OtherOrbInfo_<ContainerAllocator>::ANO_AOP_USABILITY_EXPIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OtherOrbInfo_<ContainerAllocator>::TYPE_NO_ORBIT_DATA;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OtherOrbInfo_<ContainerAllocator>::TYPE_ASSISTNOW_OFFLINE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OtherOrbInfo_<ContainerAllocator>::TYPE_ASSISTNOW_AUTONOMOUS;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__OTHER_ORB_INFO__STRUCT_HPP_
