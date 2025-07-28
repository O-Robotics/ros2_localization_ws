// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/GpsFix.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__GPS_FIX__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__GPS_FIX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__GpsFix __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__GpsFix __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GpsFix_
{
  using Type = GpsFix_<ContainerAllocator>;

  explicit GpsFix_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
    }
  }

  explicit GpsFix_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
    }
  }

  // field types and members
  using _fix_type_type =
    uint8_t;
  _fix_type_type fix_type;

  // setters for named parameter idiom
  Type & set__fix_type(
    const uint8_t & _arg)
  {
    this->fix_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t GPS_NO_FIX =
    0u;
  static constexpr uint8_t GPS_DEAD_RECKONING_ONLY =
    1u;
  static constexpr uint8_t GPS_FIX_2D =
    2u;
  static constexpr uint8_t GPS_FIX_3D =
    3u;
  static constexpr uint8_t GPS_PLUS_DEAD_RECKONING =
    4u;
  static constexpr uint8_t GPS_TIME_ONLY =
    5u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__GpsFix
    std::shared_ptr<ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__GpsFix
    std::shared_ptr<ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GpsFix_ & other) const
  {
    if (this->fix_type != other.fix_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const GpsFix_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GpsFix_

// alias to use template instance with default allocator
using GpsFix =
  ublox_ubx_msgs::msg::GpsFix_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GpsFix_<ContainerAllocator>::GPS_NO_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GpsFix_<ContainerAllocator>::GPS_DEAD_RECKONING_ONLY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GpsFix_<ContainerAllocator>::GPS_FIX_2D;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GpsFix_<ContainerAllocator>::GPS_FIX_3D;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GpsFix_<ContainerAllocator>::GPS_PLUS_DEAD_RECKONING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GpsFix_<ContainerAllocator>::GPS_TIME_ONLY;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__GPS_FIX__STRUCT_HPP_
