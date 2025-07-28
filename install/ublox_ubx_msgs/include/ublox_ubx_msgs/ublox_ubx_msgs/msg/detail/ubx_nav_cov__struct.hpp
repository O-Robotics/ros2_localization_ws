// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavCov.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavCov __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavCov __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavCov_
{
  using Type = UBXNavCov_<ContainerAllocator>;

  explicit UBXNavCov_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->version = 0;
      this->pos_cor_valid = false;
      this->vel_cor_valid = false;
      this->pos_cov_nn = 0.0f;
      this->pos_cov_ne = 0.0f;
      this->pos_cov_nd = 0.0f;
      this->pos_cov_ee = 0.0f;
      this->pos_cov_ed = 0.0f;
      this->pos_cov_dd = 0.0f;
      this->vel_cov_nn = 0.0f;
      this->vel_cov_ne = 0.0f;
      this->vel_cov_nd = 0.0f;
      this->vel_cov_ee = 0.0f;
      this->vel_cov_ed = 0.0f;
      this->vel_cov_dd = 0.0f;
    }
  }

  explicit UBXNavCov_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->version = 0;
      this->pos_cor_valid = false;
      this->vel_cor_valid = false;
      this->pos_cov_nn = 0.0f;
      this->pos_cov_ne = 0.0f;
      this->pos_cov_nd = 0.0f;
      this->pos_cov_ee = 0.0f;
      this->pos_cov_ed = 0.0f;
      this->pos_cov_dd = 0.0f;
      this->vel_cov_nn = 0.0f;
      this->vel_cov_ne = 0.0f;
      this->vel_cov_nd = 0.0f;
      this->vel_cov_ee = 0.0f;
      this->vel_cov_ed = 0.0f;
      this->vel_cov_dd = 0.0f;
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
  using _pos_cor_valid_type =
    bool;
  _pos_cor_valid_type pos_cor_valid;
  using _vel_cor_valid_type =
    bool;
  _vel_cor_valid_type vel_cor_valid;
  using _pos_cov_nn_type =
    float;
  _pos_cov_nn_type pos_cov_nn;
  using _pos_cov_ne_type =
    float;
  _pos_cov_ne_type pos_cov_ne;
  using _pos_cov_nd_type =
    float;
  _pos_cov_nd_type pos_cov_nd;
  using _pos_cov_ee_type =
    float;
  _pos_cov_ee_type pos_cov_ee;
  using _pos_cov_ed_type =
    float;
  _pos_cov_ed_type pos_cov_ed;
  using _pos_cov_dd_type =
    float;
  _pos_cov_dd_type pos_cov_dd;
  using _vel_cov_nn_type =
    float;
  _vel_cov_nn_type vel_cov_nn;
  using _vel_cov_ne_type =
    float;
  _vel_cov_ne_type vel_cov_ne;
  using _vel_cov_nd_type =
    float;
  _vel_cov_nd_type vel_cov_nd;
  using _vel_cov_ee_type =
    float;
  _vel_cov_ee_type vel_cov_ee;
  using _vel_cov_ed_type =
    float;
  _vel_cov_ed_type vel_cov_ed;
  using _vel_cov_dd_type =
    float;
  _vel_cov_dd_type vel_cov_dd;

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
  Type & set__pos_cor_valid(
    const bool & _arg)
  {
    this->pos_cor_valid = _arg;
    return *this;
  }
  Type & set__vel_cor_valid(
    const bool & _arg)
  {
    this->vel_cor_valid = _arg;
    return *this;
  }
  Type & set__pos_cov_nn(
    const float & _arg)
  {
    this->pos_cov_nn = _arg;
    return *this;
  }
  Type & set__pos_cov_ne(
    const float & _arg)
  {
    this->pos_cov_ne = _arg;
    return *this;
  }
  Type & set__pos_cov_nd(
    const float & _arg)
  {
    this->pos_cov_nd = _arg;
    return *this;
  }
  Type & set__pos_cov_ee(
    const float & _arg)
  {
    this->pos_cov_ee = _arg;
    return *this;
  }
  Type & set__pos_cov_ed(
    const float & _arg)
  {
    this->pos_cov_ed = _arg;
    return *this;
  }
  Type & set__pos_cov_dd(
    const float & _arg)
  {
    this->pos_cov_dd = _arg;
    return *this;
  }
  Type & set__vel_cov_nn(
    const float & _arg)
  {
    this->vel_cov_nn = _arg;
    return *this;
  }
  Type & set__vel_cov_ne(
    const float & _arg)
  {
    this->vel_cov_ne = _arg;
    return *this;
  }
  Type & set__vel_cov_nd(
    const float & _arg)
  {
    this->vel_cov_nd = _arg;
    return *this;
  }
  Type & set__vel_cov_ee(
    const float & _arg)
  {
    this->vel_cov_ee = _arg;
    return *this;
  }
  Type & set__vel_cov_ed(
    const float & _arg)
  {
    this->vel_cov_ed = _arg;
    return *this;
  }
  Type & set__vel_cov_dd(
    const float & _arg)
  {
    this->vel_cov_dd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavCov
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavCov
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavCov_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavCov_ & other) const
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
    if (this->pos_cor_valid != other.pos_cor_valid) {
      return false;
    }
    if (this->vel_cor_valid != other.vel_cor_valid) {
      return false;
    }
    if (this->pos_cov_nn != other.pos_cov_nn) {
      return false;
    }
    if (this->pos_cov_ne != other.pos_cov_ne) {
      return false;
    }
    if (this->pos_cov_nd != other.pos_cov_nd) {
      return false;
    }
    if (this->pos_cov_ee != other.pos_cov_ee) {
      return false;
    }
    if (this->pos_cov_ed != other.pos_cov_ed) {
      return false;
    }
    if (this->pos_cov_dd != other.pos_cov_dd) {
      return false;
    }
    if (this->vel_cov_nn != other.vel_cov_nn) {
      return false;
    }
    if (this->vel_cov_ne != other.vel_cov_ne) {
      return false;
    }
    if (this->vel_cov_nd != other.vel_cov_nd) {
      return false;
    }
    if (this->vel_cov_ee != other.vel_cov_ee) {
      return false;
    }
    if (this->vel_cov_ed != other.vel_cov_ed) {
      return false;
    }
    if (this->vel_cov_dd != other.vel_cov_dd) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavCov_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavCov_

// alias to use template instance with default allocator
using UBXNavCov =
  ublox_ubx_msgs::msg::UBXNavCov_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_COV__STRUCT_HPP_
