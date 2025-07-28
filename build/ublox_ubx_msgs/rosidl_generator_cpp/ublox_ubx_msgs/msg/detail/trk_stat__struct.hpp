// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/TrkStat.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__TrkStat __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__TrkStat __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrkStat_
{
  using Type = TrkStat_<ContainerAllocator>;

  explicit TrkStat_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pr_valid = false;
      this->cp_valid = false;
      this->half_cyc = false;
      this->sub_half_cyc = false;
    }
  }

  explicit TrkStat_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pr_valid = false;
      this->cp_valid = false;
      this->half_cyc = false;
      this->sub_half_cyc = false;
    }
  }

  // field types and members
  using _pr_valid_type =
    bool;
  _pr_valid_type pr_valid;
  using _cp_valid_type =
    bool;
  _cp_valid_type cp_valid;
  using _half_cyc_type =
    bool;
  _half_cyc_type half_cyc;
  using _sub_half_cyc_type =
    bool;
  _sub_half_cyc_type sub_half_cyc;

  // setters for named parameter idiom
  Type & set__pr_valid(
    const bool & _arg)
  {
    this->pr_valid = _arg;
    return *this;
  }
  Type & set__cp_valid(
    const bool & _arg)
  {
    this->cp_valid = _arg;
    return *this;
  }
  Type & set__half_cyc(
    const bool & _arg)
  {
    this->half_cyc = _arg;
    return *this;
  }
  Type & set__sub_half_cyc(
    const bool & _arg)
  {
    this->sub_half_cyc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__TrkStat
    std::shared_ptr<ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__TrkStat
    std::shared_ptr<ublox_ubx_msgs::msg::TrkStat_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrkStat_ & other) const
  {
    if (this->pr_valid != other.pr_valid) {
      return false;
    }
    if (this->cp_valid != other.cp_valid) {
      return false;
    }
    if (this->half_cyc != other.half_cyc) {
      return false;
    }
    if (this->sub_half_cyc != other.sub_half_cyc) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrkStat_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrkStat_

// alias to use template instance with default allocator
using TrkStat =
  ublox_ubx_msgs::msg::TrkStat_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__TRK_STAT__STRUCT_HPP_
