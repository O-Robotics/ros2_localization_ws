// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/CarrSoln.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__CARR_SOLN__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__CARR_SOLN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__CarrSoln __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__CarrSoln __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CarrSoln_
{
  using Type = CarrSoln_<ContainerAllocator>;

  explicit CarrSoln_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit CarrSoln_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CARRIER_SOLUTION_NO_CARRIER_RANGE_SOLUTION =
    0u;
  static constexpr uint8_t CARRIER_SOLUTION_PHASE_WITH_FLOATING_AMBIGUITIES =
    1u;
  static constexpr uint8_t CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES =
    2u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__CarrSoln
    std::shared_ptr<ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__CarrSoln
    std::shared_ptr<ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarrSoln_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarrSoln_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarrSoln_

// alias to use template instance with default allocator
using CarrSoln =
  ublox_ubx_msgs::msg::CarrSoln_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CarrSoln_<ContainerAllocator>::CARRIER_SOLUTION_NO_CARRIER_RANGE_SOLUTION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CarrSoln_<ContainerAllocator>::CARRIER_SOLUTION_PHASE_WITH_FLOATING_AMBIGUITIES;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CarrSoln_<ContainerAllocator>::CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__CARR_SOLN__STRUCT_HPP_
