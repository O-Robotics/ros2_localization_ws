// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/RecStat.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__REC_STAT__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__REC_STAT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__RecStat __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__RecStat __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RecStat_
{
  using Type = RecStat_<ContainerAllocator>;

  explicit RecStat_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->leap_sec = false;
      this->clk_reset = false;
    }
  }

  explicit RecStat_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->leap_sec = false;
      this->clk_reset = false;
    }
  }

  // field types and members
  using _leap_sec_type =
    bool;
  _leap_sec_type leap_sec;
  using _clk_reset_type =
    bool;
  _clk_reset_type clk_reset;

  // setters for named parameter idiom
  Type & set__leap_sec(
    const bool & _arg)
  {
    this->leap_sec = _arg;
    return *this;
  }
  Type & set__clk_reset(
    const bool & _arg)
  {
    this->clk_reset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::RecStat_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::RecStat_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::RecStat_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::RecStat_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::RecStat_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::RecStat_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::RecStat_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::RecStat_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::RecStat_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::RecStat_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__RecStat
    std::shared_ptr<ublox_ubx_msgs::msg::RecStat_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__RecStat
    std::shared_ptr<ublox_ubx_msgs::msg::RecStat_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RecStat_ & other) const
  {
    if (this->leap_sec != other.leap_sec) {
      return false;
    }
    if (this->clk_reset != other.clk_reset) {
      return false;
    }
    return true;
  }
  bool operator!=(const RecStat_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RecStat_

// alias to use template instance with default allocator
using RecStat =
  ublox_ubx_msgs::msg::RecStat_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__REC_STAT__STRUCT_HPP_
