// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/SatInfo.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'flags'
#include "ublox_ubx_msgs/msg/detail/sat_flags__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__SatInfo __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__SatInfo __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SatInfo_
{
  using Type = SatInfo_<ContainerAllocator>;

  explicit SatInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : flags(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gnss_id = 0;
      this->sv_id = 0;
      this->cno = 0;
      this->elev = 0;
      this->azim = 0;
      this->pr_res = 0;
    }
  }

  explicit SatInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : flags(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gnss_id = 0;
      this->sv_id = 0;
      this->cno = 0;
      this->elev = 0;
      this->azim = 0;
      this->pr_res = 0;
    }
  }

  // field types and members
  using _gnss_id_type =
    uint8_t;
  _gnss_id_type gnss_id;
  using _sv_id_type =
    uint8_t;
  _sv_id_type sv_id;
  using _cno_type =
    uint8_t;
  _cno_type cno;
  using _elev_type =
    int8_t;
  _elev_type elev;
  using _azim_type =
    int16_t;
  _azim_type azim;
  using _pr_res_type =
    int16_t;
  _pr_res_type pr_res;
  using _flags_type =
    ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator>;
  _flags_type flags;

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
  Type & set__cno(
    const uint8_t & _arg)
  {
    this->cno = _arg;
    return *this;
  }
  Type & set__elev(
    const int8_t & _arg)
  {
    this->elev = _arg;
    return *this;
  }
  Type & set__azim(
    const int16_t & _arg)
  {
    this->azim = _arg;
    return *this;
  }
  Type & set__pr_res(
    const int16_t & _arg)
  {
    this->pr_res = _arg;
    return *this;
  }
  Type & set__flags(
    const ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator> & _arg)
  {
    this->flags = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__SatInfo
    std::shared_ptr<ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__SatInfo
    std::shared_ptr<ublox_ubx_msgs::msg::SatInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SatInfo_ & other) const
  {
    if (this->gnss_id != other.gnss_id) {
      return false;
    }
    if (this->sv_id != other.sv_id) {
      return false;
    }
    if (this->cno != other.cno) {
      return false;
    }
    if (this->elev != other.elev) {
      return false;
    }
    if (this->azim != other.azim) {
      return false;
    }
    if (this->pr_res != other.pr_res) {
      return false;
    }
    if (this->flags != other.flags) {
      return false;
    }
    return true;
  }
  bool operator!=(const SatInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SatInfo_

// alias to use template instance with default allocator
using SatInfo =
  ublox_ubx_msgs::msg::SatInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SAT_INFO__STRUCT_HPP_
