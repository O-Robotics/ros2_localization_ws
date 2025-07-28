// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavDOP.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavDOP __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavDOP __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavDOP_
{
  using Type = UBXNavDOP_<ContainerAllocator>;

  explicit UBXNavDOP_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->g_dop = 0ul;
      this->p_dop = 0ul;
      this->t_dop = 0ul;
      this->v_dop = 0ul;
      this->h_dop = 0ul;
      this->n_dop = 0ul;
      this->e_dop = 0ul;
    }
  }

  explicit UBXNavDOP_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->g_dop = 0ul;
      this->p_dop = 0ul;
      this->t_dop = 0ul;
      this->v_dop = 0ul;
      this->h_dop = 0ul;
      this->n_dop = 0ul;
      this->e_dop = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _g_dop_type =
    uint32_t;
  _g_dop_type g_dop;
  using _p_dop_type =
    uint32_t;
  _p_dop_type p_dop;
  using _t_dop_type =
    uint32_t;
  _t_dop_type t_dop;
  using _v_dop_type =
    uint32_t;
  _v_dop_type v_dop;
  using _h_dop_type =
    uint32_t;
  _h_dop_type h_dop;
  using _n_dop_type =
    uint32_t;
  _n_dop_type n_dop;
  using _e_dop_type =
    uint32_t;
  _e_dop_type e_dop;

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
  Type & set__g_dop(
    const uint32_t & _arg)
  {
    this->g_dop = _arg;
    return *this;
  }
  Type & set__p_dop(
    const uint32_t & _arg)
  {
    this->p_dop = _arg;
    return *this;
  }
  Type & set__t_dop(
    const uint32_t & _arg)
  {
    this->t_dop = _arg;
    return *this;
  }
  Type & set__v_dop(
    const uint32_t & _arg)
  {
    this->v_dop = _arg;
    return *this;
  }
  Type & set__h_dop(
    const uint32_t & _arg)
  {
    this->h_dop = _arg;
    return *this;
  }
  Type & set__n_dop(
    const uint32_t & _arg)
  {
    this->n_dop = _arg;
    return *this;
  }
  Type & set__e_dop(
    const uint32_t & _arg)
  {
    this->e_dop = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavDOP
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavDOP
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavDOP_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavDOP_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->g_dop != other.g_dop) {
      return false;
    }
    if (this->p_dop != other.p_dop) {
      return false;
    }
    if (this->t_dop != other.t_dop) {
      return false;
    }
    if (this->v_dop != other.v_dop) {
      return false;
    }
    if (this->h_dop != other.h_dop) {
      return false;
    }
    if (this->n_dop != other.n_dop) {
      return false;
    }
    if (this->e_dop != other.e_dop) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavDOP_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavDOP_

// alias to use template instance with default allocator
using UBXNavDOP =
  ublox_ubx_msgs::msg::UBXNavDOP_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_DOP__STRUCT_HPP_
