// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavClock.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavClock __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavClock __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavClock_
{
  using Type = UBXNavClock_<ContainerAllocator>;

  explicit UBXNavClock_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->clk_b = 0l;
      this->clk_d = 0l;
      this->t_acc = 0ul;
      this->f_acc = 0ul;
    }
  }

  explicit UBXNavClock_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->clk_b = 0l;
      this->clk_d = 0l;
      this->t_acc = 0ul;
      this->f_acc = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _clk_b_type =
    int32_t;
  _clk_b_type clk_b;
  using _clk_d_type =
    int32_t;
  _clk_d_type clk_d;
  using _t_acc_type =
    uint32_t;
  _t_acc_type t_acc;
  using _f_acc_type =
    uint32_t;
  _f_acc_type f_acc;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__itow(
    const uint32_t & _arg)
  {
    this->itow = _arg;
    return *this;
  }
  Type & set__clk_b(
    const int32_t & _arg)
  {
    this->clk_b = _arg;
    return *this;
  }
  Type & set__clk_d(
    const int32_t & _arg)
  {
    this->clk_d = _arg;
    return *this;
  }
  Type & set__t_acc(
    const uint32_t & _arg)
  {
    this->t_acc = _arg;
    return *this;
  }
  Type & set__f_acc(
    const uint32_t & _arg)
  {
    this->f_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavClock
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavClock
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavClock_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavClock_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->clk_b != other.clk_b) {
      return false;
    }
    if (this->clk_d != other.clk_d) {
      return false;
    }
    if (this->t_acc != other.t_acc) {
      return false;
    }
    if (this->f_acc != other.f_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavClock_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavClock_

// alias to use template instance with default allocator
using UBXNavClock =
  ublox_ubx_msgs::msg::UBXNavClock_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_CLOCK__STRUCT_HPP_
