// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavHPPosLLH.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__STRUCT_HPP_

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
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavHPPosLLH __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavHPPosLLH __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavHPPosLLH_
{
  using Type = UBXNavHPPosLLH_<ContainerAllocator>;

  explicit UBXNavHPPosLLH_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->invalid_lon = false;
      this->invalid_lat = false;
      this->invalid_height = false;
      this->invalid_hmsl = false;
      this->invalid_lon_hp = false;
      this->invalid_lat_hp = false;
      this->invalid_height_hp = false;
      this->invalid_hmsl_hp = false;
      this->itow = 0ul;
      this->lon = 0l;
      this->lat = 0l;
      this->height = 0l;
      this->hmsl = 0l;
      this->lon_hp = 0;
      this->lat_hp = 0;
      this->height_hp = 0;
      this->hmsl_hp = 0;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
    }
  }

  explicit UBXNavHPPosLLH_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->invalid_lon = false;
      this->invalid_lat = false;
      this->invalid_height = false;
      this->invalid_hmsl = false;
      this->invalid_lon_hp = false;
      this->invalid_lat_hp = false;
      this->invalid_height_hp = false;
      this->invalid_hmsl_hp = false;
      this->itow = 0ul;
      this->lon = 0l;
      this->lat = 0l;
      this->height = 0l;
      this->hmsl = 0l;
      this->lon_hp = 0;
      this->lat_hp = 0;
      this->height_hp = 0;
      this->hmsl_hp = 0;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _version_type =
    uint8_t;
  _version_type version;
  using _invalid_lon_type =
    bool;
  _invalid_lon_type invalid_lon;
  using _invalid_lat_type =
    bool;
  _invalid_lat_type invalid_lat;
  using _invalid_height_type =
    bool;
  _invalid_height_type invalid_height;
  using _invalid_hmsl_type =
    bool;
  _invalid_hmsl_type invalid_hmsl;
  using _invalid_lon_hp_type =
    bool;
  _invalid_lon_hp_type invalid_lon_hp;
  using _invalid_lat_hp_type =
    bool;
  _invalid_lat_hp_type invalid_lat_hp;
  using _invalid_height_hp_type =
    bool;
  _invalid_height_hp_type invalid_height_hp;
  using _invalid_hmsl_hp_type =
    bool;
  _invalid_hmsl_hp_type invalid_hmsl_hp;
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
  using _lon_hp_type =
    int8_t;
  _lon_hp_type lon_hp;
  using _lat_hp_type =
    int8_t;
  _lat_hp_type lat_hp;
  using _height_hp_type =
    int8_t;
  _height_hp_type height_hp;
  using _hmsl_hp_type =
    int8_t;
  _hmsl_hp_type hmsl_hp;
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
  Type & set__version(
    const uint8_t & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__invalid_lon(
    const bool & _arg)
  {
    this->invalid_lon = _arg;
    return *this;
  }
  Type & set__invalid_lat(
    const bool & _arg)
  {
    this->invalid_lat = _arg;
    return *this;
  }
  Type & set__invalid_height(
    const bool & _arg)
  {
    this->invalid_height = _arg;
    return *this;
  }
  Type & set__invalid_hmsl(
    const bool & _arg)
  {
    this->invalid_hmsl = _arg;
    return *this;
  }
  Type & set__invalid_lon_hp(
    const bool & _arg)
  {
    this->invalid_lon_hp = _arg;
    return *this;
  }
  Type & set__invalid_lat_hp(
    const bool & _arg)
  {
    this->invalid_lat_hp = _arg;
    return *this;
  }
  Type & set__invalid_height_hp(
    const bool & _arg)
  {
    this->invalid_height_hp = _arg;
    return *this;
  }
  Type & set__invalid_hmsl_hp(
    const bool & _arg)
  {
    this->invalid_hmsl_hp = _arg;
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
  Type & set__lon_hp(
    const int8_t & _arg)
  {
    this->lon_hp = _arg;
    return *this;
  }
  Type & set__lat_hp(
    const int8_t & _arg)
  {
    this->lat_hp = _arg;
    return *this;
  }
  Type & set__height_hp(
    const int8_t & _arg)
  {
    this->height_hp = _arg;
    return *this;
  }
  Type & set__hmsl_hp(
    const int8_t & _arg)
  {
    this->hmsl_hp = _arg;
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
    ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavHPPosLLH
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavHPPosLLH
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavHPPosLLH_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavHPPosLLH_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->invalid_lon != other.invalid_lon) {
      return false;
    }
    if (this->invalid_lat != other.invalid_lat) {
      return false;
    }
    if (this->invalid_height != other.invalid_height) {
      return false;
    }
    if (this->invalid_hmsl != other.invalid_hmsl) {
      return false;
    }
    if (this->invalid_lon_hp != other.invalid_lon_hp) {
      return false;
    }
    if (this->invalid_lat_hp != other.invalid_lat_hp) {
      return false;
    }
    if (this->invalid_height_hp != other.invalid_height_hp) {
      return false;
    }
    if (this->invalid_hmsl_hp != other.invalid_hmsl_hp) {
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
    if (this->lon_hp != other.lon_hp) {
      return false;
    }
    if (this->lat_hp != other.lat_hp) {
      return false;
    }
    if (this->height_hp != other.height_hp) {
      return false;
    }
    if (this->hmsl_hp != other.hmsl_hp) {
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
  bool operator!=(const UBXNavHPPosLLH_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavHPPosLLH_

// alias to use template instance with default allocator
using UBXNavHPPosLLH =
  ublox_ubx_msgs::msg::UBXNavHPPosLLH_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_HP_POS_LLH__STRUCT_HPP_
