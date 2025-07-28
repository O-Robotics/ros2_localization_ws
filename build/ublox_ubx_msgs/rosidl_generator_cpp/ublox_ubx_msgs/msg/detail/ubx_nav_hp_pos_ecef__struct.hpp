// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosECEF.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavHPPosECEF __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavHPPosECEF __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavHPPosECEF_
{
  using Type = UBXNavHPPosECEF_<ContainerAllocator>;

  explicit UBXNavHPPosECEF_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->itow = 0ul;
      this->ecef_x = 0l;
      this->ecef_y = 0l;
      this->ecef_z = 0l;
      this->ecef_x_hp = 0;
      this->ecef_y_hp = 0;
      this->ecef_z_hp = 0;
      this->invalid_ecef_x = false;
      this->invalid_ecef_y = false;
      this->invalid_ecef_z = false;
      this->invalid_ecef_x_hp = false;
      this->invalid_ecef_y_hp = false;
      this->invalid_ecef_z_hp = false;
      this->p_acc = 0ul;
    }
  }

  explicit UBXNavHPPosECEF_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->itow = 0ul;
      this->ecef_x = 0l;
      this->ecef_y = 0l;
      this->ecef_z = 0l;
      this->ecef_x_hp = 0;
      this->ecef_y_hp = 0;
      this->ecef_z_hp = 0;
      this->invalid_ecef_x = false;
      this->invalid_ecef_y = false;
      this->invalid_ecef_z = false;
      this->invalid_ecef_x_hp = false;
      this->invalid_ecef_y_hp = false;
      this->invalid_ecef_z_hp = false;
      this->p_acc = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _version_type =
    uint8_t;
  _version_type version;
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
  using _ecef_x_hp_type =
    int8_t;
  _ecef_x_hp_type ecef_x_hp;
  using _ecef_y_hp_type =
    int8_t;
  _ecef_y_hp_type ecef_y_hp;
  using _ecef_z_hp_type =
    int8_t;
  _ecef_z_hp_type ecef_z_hp;
  using _invalid_ecef_x_type =
    bool;
  _invalid_ecef_x_type invalid_ecef_x;
  using _invalid_ecef_y_type =
    bool;
  _invalid_ecef_y_type invalid_ecef_y;
  using _invalid_ecef_z_type =
    bool;
  _invalid_ecef_z_type invalid_ecef_z;
  using _invalid_ecef_x_hp_type =
    bool;
  _invalid_ecef_x_hp_type invalid_ecef_x_hp;
  using _invalid_ecef_y_hp_type =
    bool;
  _invalid_ecef_y_hp_type invalid_ecef_y_hp;
  using _invalid_ecef_z_hp_type =
    bool;
  _invalid_ecef_z_hp_type invalid_ecef_z_hp;
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
  Type & set__version(
    const uint8_t & _arg)
  {
    this->version = _arg;
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
  Type & set__ecef_x_hp(
    const int8_t & _arg)
  {
    this->ecef_x_hp = _arg;
    return *this;
  }
  Type & set__ecef_y_hp(
    const int8_t & _arg)
  {
    this->ecef_y_hp = _arg;
    return *this;
  }
  Type & set__ecef_z_hp(
    const int8_t & _arg)
  {
    this->ecef_z_hp = _arg;
    return *this;
  }
  Type & set__invalid_ecef_x(
    const bool & _arg)
  {
    this->invalid_ecef_x = _arg;
    return *this;
  }
  Type & set__invalid_ecef_y(
    const bool & _arg)
  {
    this->invalid_ecef_y = _arg;
    return *this;
  }
  Type & set__invalid_ecef_z(
    const bool & _arg)
  {
    this->invalid_ecef_z = _arg;
    return *this;
  }
  Type & set__invalid_ecef_x_hp(
    const bool & _arg)
  {
    this->invalid_ecef_x_hp = _arg;
    return *this;
  }
  Type & set__invalid_ecef_y_hp(
    const bool & _arg)
  {
    this->invalid_ecef_y_hp = _arg;
    return *this;
  }
  Type & set__invalid_ecef_z_hp(
    const bool & _arg)
  {
    this->invalid_ecef_z_hp = _arg;
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
    ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavHPPosECEF
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavHPPosECEF
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavHPPosECEF_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavHPPosECEF_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->version != other.version) {
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
    if (this->ecef_x_hp != other.ecef_x_hp) {
      return false;
    }
    if (this->ecef_y_hp != other.ecef_y_hp) {
      return false;
    }
    if (this->ecef_z_hp != other.ecef_z_hp) {
      return false;
    }
    if (this->invalid_ecef_x != other.invalid_ecef_x) {
      return false;
    }
    if (this->invalid_ecef_y != other.invalid_ecef_y) {
      return false;
    }
    if (this->invalid_ecef_z != other.invalid_ecef_z) {
      return false;
    }
    if (this->invalid_ecef_x_hp != other.invalid_ecef_x_hp) {
      return false;
    }
    if (this->invalid_ecef_y_hp != other.invalid_ecef_y_hp) {
      return false;
    }
    if (this->invalid_ecef_z_hp != other.invalid_ecef_z_hp) {
      return false;
    }
    if (this->p_acc != other.p_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavHPPosECEF_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavHPPosECEF_

// alias to use template instance with default allocator
using UBXNavHPPosECEF =
  ublox_ubx_msgs::msg::UBXNavHPPosECEF_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_ECEF__STRUCT_HPP_
