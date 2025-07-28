// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPosECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavPosECEF __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavPosECEF __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavPosECEF_
{
  using Type = UBXNavPosECEF_<ContainerAllocator>;

  explicit UBXNavPosECEF_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->ecef_x = 0l;
      this->ecef_y = 0l;
      this->ecef_z = 0l;
      this->p_acc = 0ul;
    }
  }

  explicit UBXNavPosECEF_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->ecef_x = 0l;
      this->ecef_y = 0l;
      this->ecef_z = 0l;
      this->p_acc = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _ecef_x_type =
    int32_t;
  _ecef_x_type ecef_x;
  using _ecef_y_type =
    int32_t;
  _ecef_y_type ecef_y;
  using _ecef_z_type =
    int32_t;
  _ecef_z_type ecef_z;
  using _p_acc_type =
    uint32_t;
  _p_acc_type p_acc;

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
  Type & set__ecef_x(
    const int32_t & _arg)
  {
    this->ecef_x = _arg;
    return *this;
  }
  Type & set__ecef_y(
    const int32_t & _arg)
  {
    this->ecef_y = _arg;
    return *this;
  }
  Type & set__ecef_z(
    const int32_t & _arg)
  {
    this->ecef_z = _arg;
    return *this;
  }
  Type & set__p_acc(
    const uint32_t & _arg)
  {
    this->p_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavPosECEF
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavPosECEF
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPosECEF_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavPosECEF_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->ecef_x != other.ecef_x) {
      return false;
    }
    if (this->ecef_y != other.ecef_y) {
      return false;
    }
    if (this->ecef_z != other.ecef_z) {
      return false;
    }
    if (this->p_acc != other.p_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavPosECEF_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavPosECEF_

// alias to use template instance with default allocator
using UBXNavPosECEF =
  ublox_ubx_msgs::msg::UBXNavPosECEF_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_ECEF__STRUCT_HPP_
