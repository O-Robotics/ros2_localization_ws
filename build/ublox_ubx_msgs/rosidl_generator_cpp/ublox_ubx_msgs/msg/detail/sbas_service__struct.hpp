// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/SBASService.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__SBASService __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__SBASService __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SBASService_
{
  using Type = SBASService_<ContainerAllocator>;

  explicit SBASService_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ranging = false;
      this->corrections = false;
      this->integrity = false;
      this->test_mode = false;
      this->bad = false;
    }
  }

  explicit SBASService_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ranging = false;
      this->corrections = false;
      this->integrity = false;
      this->test_mode = false;
      this->bad = false;
    }
  }

  // field types and members
  using _ranging_type =
    bool;
  _ranging_type ranging;
  using _corrections_type =
    bool;
  _corrections_type corrections;
  using _integrity_type =
    bool;
  _integrity_type integrity;
  using _test_mode_type =
    bool;
  _test_mode_type test_mode;
  using _bad_type =
    bool;
  _bad_type bad;

  // setters for named parameter idiom
  Type & set__ranging(
    const bool & _arg)
  {
    this->ranging = _arg;
    return *this;
  }
  Type & set__corrections(
    const bool & _arg)
  {
    this->corrections = _arg;
    return *this;
  }
  Type & set__integrity(
    const bool & _arg)
  {
    this->integrity = _arg;
    return *this;
  }
  Type & set__test_mode(
    const bool & _arg)
  {
    this->test_mode = _arg;
    return *this;
  }
  Type & set__bad(
    const bool & _arg)
  {
    this->bad = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::SBASService_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::SBASService_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SBASService_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SBASService_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SBASService_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SBASService_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SBASService_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SBASService_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SBASService_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SBASService_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__SBASService
    std::shared_ptr<ublox_ubx_msgs::msg::SBASService_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__SBASService
    std::shared_ptr<ublox_ubx_msgs::msg::SBASService_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SBASService_ & other) const
  {
    if (this->ranging != other.ranging) {
      return false;
    }
    if (this->corrections != other.corrections) {
      return false;
    }
    if (this->integrity != other.integrity) {
      return false;
    }
    if (this->test_mode != other.test_mode) {
      return false;
    }
    if (this->bad != other.bad) {
      return false;
    }
    return true;
  }
  bool operator!=(const SBASService_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SBASService_

// alias to use template instance with default allocator
using SBASService =
  ublox_ubx_msgs::msg::SBASService_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SERVICE__STRUCT_HPP_
