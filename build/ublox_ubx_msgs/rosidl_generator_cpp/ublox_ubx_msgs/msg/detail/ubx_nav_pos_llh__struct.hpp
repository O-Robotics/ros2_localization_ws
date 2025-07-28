// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavPosLLH.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_LLH__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_LLH__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavPosLLH __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavPosLLH __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavPosLLH_
{
  using Type = UBXNavPosLLH_<ContainerAllocator>;

  explicit UBXNavPosLLH_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->lon = 0l;
      this->lat = 0l;
      this->height = 0l;
      this->hmsl = 0l;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
    }
  }

  explicit UBXNavPosLLH_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->lon = 0l;
      this->lat = 0l;
      this->height = 0l;
      this->hmsl = 0l;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _lon_type =
    int32_t;
  _lon_type lon;
  using _lat_type =
    int32_t;
  _lat_type lat;
  using _height_type =
    int32_t;
  _height_type height;
  using _hmsl_type =
    int32_t;
  _hmsl_type hmsl;
  using _h_acc_type =
    uint32_t;
  _h_acc_type h_acc;
  using _v_acc_type =
    uint32_t;
  _v_acc_type v_acc;

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
  Type & set__lon(
    const int32_t & _arg)
  {
    this->lon = _arg;
    return *this;
  }
  Type & set__lat(
    const int32_t & _arg)
  {
    this->lat = _arg;
    return *this;
  }
  Type & set__height(
    const int32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__hmsl(
    const int32_t & _arg)
  {
    this->hmsl = _arg;
    return *this;
  }
  Type & set__h_acc(
    const uint32_t & _arg)
  {
    this->h_acc = _arg;
    return *this;
  }
  Type & set__v_acc(
    const uint32_t & _arg)
  {
    this->v_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavPosLLH
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavPosLLH
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavPosLLH_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavPosLLH_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    if (this->lat != other.lat) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->hmsl != other.hmsl) {
      return false;
    }
    if (this->h_acc != other.h_acc) {
      return false;
    }
    if (this->v_acc != other.v_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavPosLLH_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavPosLLH_

// alias to use template instance with default allocator
using UBXNavPosLLH =
  ublox_ubx_msgs::msg::UBXNavPosLLH_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_POS_LLH__STRUCT_HPP_
