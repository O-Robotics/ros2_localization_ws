// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavVelECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavVelECEF __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavVelECEF __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavVelECEF_
{
  using Type = UBXNavVelECEF_<ContainerAllocator>;

  explicit UBXNavVelECEF_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->ecef_vx = 0l;
      this->ecef_vy = 0l;
      this->ecef_vz = 0l;
      this->s_acc = 0ul;
    }
  }

  explicit UBXNavVelECEF_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->ecef_vx = 0l;
      this->ecef_vy = 0l;
      this->ecef_vz = 0l;
      this->s_acc = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _ecef_vx_type =
    int32_t;
  _ecef_vx_type ecef_vx;
  using _ecef_vy_type =
    int32_t;
  _ecef_vy_type ecef_vy;
  using _ecef_vz_type =
    int32_t;
  _ecef_vz_type ecef_vz;
  using _s_acc_type =
    uint32_t;
  _s_acc_type s_acc;

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
  Type & set__ecef_vx(
    const int32_t & _arg)
  {
    this->ecef_vx = _arg;
    return *this;
  }
  Type & set__ecef_vy(
    const int32_t & _arg)
  {
    this->ecef_vy = _arg;
    return *this;
  }
  Type & set__ecef_vz(
    const int32_t & _arg)
  {
    this->ecef_vz = _arg;
    return *this;
  }
  Type & set__s_acc(
    const uint32_t & _arg)
  {
    this->s_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavVelECEF
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavVelECEF
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavVelECEF_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavVelECEF_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->ecef_vx != other.ecef_vx) {
      return false;
    }
    if (this->ecef_vy != other.ecef_vy) {
      return false;
    }
    if (this->ecef_vz != other.ecef_vz) {
      return false;
    }
    if (this->s_acc != other.s_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavVelECEF_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavVelECEF_

// alias to use template instance with default allocator
using UBXNavVelECEF =
  ublox_ubx_msgs::msg::UBXNavVelECEF_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_VEL_ECEF__STRUCT_HPP_
