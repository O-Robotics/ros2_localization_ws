// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavVelNED.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavVelNED __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavVelNED __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavVelNED_
{
  using Type = UBXNavVelNED_<ContainerAllocator>;

  explicit UBXNavVelNED_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->vel_n = 0l;
      this->vel_e = 0l;
      this->vel_d = 0l;
      this->speed = 0ul;
      this->g_speed = 0ul;
      this->heading = 0l;
      this->s_acc = 0ul;
      this->c_acc = 0ul;
    }
  }

  explicit UBXNavVelNED_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->vel_n = 0l;
      this->vel_e = 0l;
      this->vel_d = 0l;
      this->speed = 0ul;
      this->g_speed = 0ul;
      this->heading = 0l;
      this->s_acc = 0ul;
      this->c_acc = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _vel_n_type =
    int32_t;
  _vel_n_type vel_n;
  using _vel_e_type =
    int32_t;
  _vel_e_type vel_e;
  using _vel_d_type =
    int32_t;
  _vel_d_type vel_d;
  using _speed_type =
    uint32_t;
  _speed_type speed;
  using _g_speed_type =
    uint32_t;
  _g_speed_type g_speed;
  using _heading_type =
    int32_t;
  _heading_type heading;
  using _s_acc_type =
    uint32_t;
  _s_acc_type s_acc;
  using _c_acc_type =
    uint32_t;
  _c_acc_type c_acc;

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
  Type & set__vel_n(
    const int32_t & _arg)
  {
    this->vel_n = _arg;
    return *this;
  }
  Type & set__vel_e(
    const int32_t & _arg)
  {
    this->vel_e = _arg;
    return *this;
  }
  Type & set__vel_d(
    const int32_t & _arg)
  {
    this->vel_d = _arg;
    return *this;
  }
  Type & set__speed(
    const uint32_t & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__g_speed(
    const uint32_t & _arg)
  {
    this->g_speed = _arg;
    return *this;
  }
  Type & set__heading(
    const int32_t & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__s_acc(
    const uint32_t & _arg)
  {
    this->s_acc = _arg;
    return *this;
  }
  Type & set__c_acc(
    const uint32_t & _arg)
  {
    this->c_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavVelNED
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavVelNED
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavVelNED_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavVelNED_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->vel_n != other.vel_n) {
      return false;
    }
    if (this->vel_e != other.vel_e) {
      return false;
    }
    if (this->vel_d != other.vel_d) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->g_speed != other.g_speed) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->s_acc != other.s_acc) {
      return false;
    }
    if (this->c_acc != other.c_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavVelNED_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavVelNED_

// alias to use template instance with default allocator
using UBXNavVelNED =
  ublox_ubx_msgs::msg::UBXNavVelNED_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_NED__STRUCT_HPP_
