// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/SBASSvData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__SBASSvData __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__SBASSvData __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SBASSvData_
{
  using Type = SBASSvData_<ContainerAllocator>;

  explicit SBASSvData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->svid = 0;
      this->reserved_1 = 0;
      this->udre = 0;
      this->sv_sys = 0;
      this->sv_service = 0;
      this->reserved_2 = 0;
      this->prc = 0;
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->reserved_3.begin(), this->reserved_3.end(), 0);
      this->ic = 0;
    }
  }

  explicit SBASSvData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reserved_3(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->svid = 0;
      this->reserved_1 = 0;
      this->udre = 0;
      this->sv_sys = 0;
      this->sv_service = 0;
      this->reserved_2 = 0;
      this->prc = 0;
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->reserved_3.begin(), this->reserved_3.end(), 0);
      this->ic = 0;
    }
  }

  // field types and members
  using _svid_type =
    uint8_t;
  _svid_type svid;
  using _reserved_1_type =
    uint8_t;
  _reserved_1_type reserved_1;
  using _udre_type =
    uint8_t;
  _udre_type udre;
  using _sv_sys_type =
    uint8_t;
  _sv_sys_type sv_sys;
  using _sv_service_type =
    uint8_t;
  _sv_service_type sv_service;
  using _reserved_2_type =
    uint8_t;
  _reserved_2_type reserved_2;
  using _prc_type =
    int16_t;
  _prc_type prc;
  using _reserved_3_type =
    std::array<uint8_t, 2>;
  _reserved_3_type reserved_3;
  using _ic_type =
    int16_t;
  _ic_type ic;

  // setters for named parameter idiom
  Type & set__svid(
    const uint8_t & _arg)
  {
    this->svid = _arg;
    return *this;
  }
  Type & set__reserved_1(
    const uint8_t & _arg)
  {
    this->reserved_1 = _arg;
    return *this;
  }
  Type & set__udre(
    const uint8_t & _arg)
  {
    this->udre = _arg;
    return *this;
  }
  Type & set__sv_sys(
    const uint8_t & _arg)
  {
    this->sv_sys = _arg;
    return *this;
  }
  Type & set__sv_service(
    const uint8_t & _arg)
  {
    this->sv_service = _arg;
    return *this;
  }
  Type & set__reserved_2(
    const uint8_t & _arg)
  {
    this->reserved_2 = _arg;
    return *this;
  }
  Type & set__prc(
    const int16_t & _arg)
  {
    this->prc = _arg;
    return *this;
  }
  Type & set__reserved_3(
    const std::array<uint8_t, 2> & _arg)
  {
    this->reserved_3 = _arg;
    return *this;
  }
  Type & set__ic(
    const int16_t & _arg)
  {
    this->ic = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__SBASSvData
    std::shared_ptr<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__SBASSvData
    std::shared_ptr<ublox_ubx_msgs::msg::SBASSvData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SBASSvData_ & other) const
  {
    if (this->svid != other.svid) {
      return false;
    }
    if (this->reserved_1 != other.reserved_1) {
      return false;
    }
    if (this->udre != other.udre) {
      return false;
    }
    if (this->sv_sys != other.sv_sys) {
      return false;
    }
    if (this->sv_service != other.sv_service) {
      return false;
    }
    if (this->reserved_2 != other.reserved_2) {
      return false;
    }
    if (this->prc != other.prc) {
      return false;
    }
    if (this->reserved_3 != other.reserved_3) {
      return false;
    }
    if (this->ic != other.ic) {
      return false;
    }
    return true;
  }
  bool operator!=(const SBASSvData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SBASSvData_

// alias to use template instance with default allocator
using SBASSvData =
  ublox_ubx_msgs::msg::SBASSvData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SBAS_SV_DATA__STRUCT_HPP_
