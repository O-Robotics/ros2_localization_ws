// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavOrb.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__STRUCT_HPP_

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
// Member 'sv_info'
#include "ublox_ubx_msgs/msg/detail/orb_sv_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavOrb __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavOrb __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavOrb_
{
  using Type = UBXNavOrb_<ContainerAllocator>;

  explicit UBXNavOrb_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->version = 0;
      this->num_sv = 0;
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->reserved_0.begin(), this->reserved_0.end(), 0);
    }
  }

  explicit UBXNavOrb_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    reserved_0(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->version = 0;
      this->num_sv = 0;
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->reserved_0.begin(), this->reserved_0.end(), 0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _version_type =
    uint8_t;
  _version_type version;
  using _num_sv_type =
    uint8_t;
  _num_sv_type num_sv;
  using _reserved_0_type =
    std::array<uint8_t, 2>;
  _reserved_0_type reserved_0;
  using _sv_info_type =
    std::vector<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>>>;
  _sv_info_type sv_info;

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
  Type & set__version(
    const uint8_t & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__num_sv(
    const uint8_t & _arg)
  {
    this->num_sv = _arg;
    return *this;
  }
  Type & set__reserved_0(
    const std::array<uint8_t, 2> & _arg)
  {
    this->reserved_0 = _arg;
    return *this;
  }
  Type & set__sv_info(
    const std::vector<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>>> & _arg)
  {
    this->sv_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavOrb
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavOrb
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavOrb_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavOrb_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->num_sv != other.num_sv) {
      return false;
    }
    if (this->reserved_0 != other.reserved_0) {
      return false;
    }
    if (this->sv_info != other.sv_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavOrb_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavOrb_

// alias to use template instance with default allocator
using UBXNavOrb =
  ublox_ubx_msgs::msg::UBXNavOrb_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_ORB__STRUCT_HPP_
