// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/OrbSVInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'sv_flag'
#include "ublox_ubx_msgs/msg/detail/orb_sv_flag__struct.hpp"
// Member 'eph'
#include "ublox_ubx_msgs/msg/detail/orb_eph_info__struct.hpp"
// Member 'alm'
#include "ublox_ubx_msgs/msg/detail/orb_alm_info__struct.hpp"
// Member 'other_orb'
#include "ublox_ubx_msgs/msg/detail/other_orb_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__OrbSVInfo __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__OrbSVInfo __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OrbSVInfo_
{
  using Type = OrbSVInfo_<ContainerAllocator>;

  explicit OrbSVInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sv_flag(_init),
    eph(_init),
    alm(_init),
    other_orb(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gnss_id = 0;
      this->sv_id = 0;
    }
  }

  explicit OrbSVInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sv_flag(_alloc, _init),
    eph(_alloc, _init),
    alm(_alloc, _init),
    other_orb(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gnss_id = 0;
      this->sv_id = 0;
    }
  }

  // field types and members
  using _gnss_id_type =
    uint8_t;
  _gnss_id_type gnss_id;
  using _sv_id_type =
    uint8_t;
  _sv_id_type sv_id;
  using _sv_flag_type =
    ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator>;
  _sv_flag_type sv_flag;
  using _eph_type =
    ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator>;
  _eph_type eph;
  using _alm_type =
    ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator>;
  _alm_type alm;
  using _other_orb_type =
    ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator>;
  _other_orb_type other_orb;

  // setters for named parameter idiom
  Type & set__gnss_id(
    const uint8_t & _arg)
  {
    this->gnss_id = _arg;
    return *this;
  }
  Type & set__sv_id(
    const uint8_t & _arg)
  {
    this->sv_id = _arg;
    return *this;
  }
  Type & set__sv_flag(
    const ublox_ubx_msgs::msg::OrbSVFlag_<ContainerAllocator> & _arg)
  {
    this->sv_flag = _arg;
    return *this;
  }
  Type & set__eph(
    const ublox_ubx_msgs::msg::OrbEphInfo_<ContainerAllocator> & _arg)
  {
    this->eph = _arg;
    return *this;
  }
  Type & set__alm(
    const ublox_ubx_msgs::msg::OrbAlmInfo_<ContainerAllocator> & _arg)
  {
    this->alm = _arg;
    return *this;
  }
  Type & set__other_orb(
    const ublox_ubx_msgs::msg::OtherOrbInfo_<ContainerAllocator> & _arg)
  {
    this->other_orb = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__OrbSVInfo
    std::shared_ptr<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__OrbSVInfo
    std::shared_ptr<ublox_ubx_msgs::msg::OrbSVInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OrbSVInfo_ & other) const
  {
    if (this->gnss_id != other.gnss_id) {
      return false;
    }
    if (this->sv_id != other.sv_id) {
      return false;
    }
    if (this->sv_flag != other.sv_flag) {
      return false;
    }
    if (this->eph != other.eph) {
      return false;
    }
    if (this->alm != other.alm) {
      return false;
    }
    if (this->other_orb != other.other_orb) {
      return false;
    }
    return true;
  }
  bool operator!=(const OrbSVInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OrbSVInfo_

// alias to use template instance with default allocator
using OrbSVInfo =
  ublox_ubx_msgs::msg::OrbSVInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ORB_SV_INFO__STRUCT_HPP_
