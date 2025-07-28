// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavRelPosNED.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__STRUCT_HPP_

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
// Member 'carr_soln'
#include "ublox_ubx_msgs/msg/detail/carr_soln__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavRelPosNED __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavRelPosNED __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavRelPosNED_
{
  using Type = UBXNavRelPosNED_<ContainerAllocator>;

  explicit UBXNavRelPosNED_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    carr_soln(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->ref_station_id = 0;
      this->itow = 0ul;
      this->rel_pos_n = 0l;
      this->rel_pos_e = 0l;
      this->rel_pos_d = 0l;
      this->rel_pos_length = 0l;
      this->rel_pos_heading = 0l;
      this->rel_pos_hp_n = 0;
      this->rel_pos_hp_e = 0;
      this->rel_pos_hp_d = 0;
      this->rel_pos_hp_length = 0;
      this->acc_n = 0ul;
      this->acc_e = 0ul;
      this->acc_d = 0ul;
      this->acc_length = 0ul;
      this->acc_heading = 0ul;
      this->gnss_fix_ok = false;
      this->diff_soln = false;
      this->rel_pos_valid = false;
      this->is_moving = false;
      this->ref_pos_miss = false;
      this->ref_obs_miss = false;
      this->rel_pos_heading_valid = false;
      this->rel_pos_normalized = false;
    }
  }

  explicit UBXNavRelPosNED_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    carr_soln(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->ref_station_id = 0;
      this->itow = 0ul;
      this->rel_pos_n = 0l;
      this->rel_pos_e = 0l;
      this->rel_pos_d = 0l;
      this->rel_pos_length = 0l;
      this->rel_pos_heading = 0l;
      this->rel_pos_hp_n = 0;
      this->rel_pos_hp_e = 0;
      this->rel_pos_hp_d = 0;
      this->rel_pos_hp_length = 0;
      this->acc_n = 0ul;
      this->acc_e = 0ul;
      this->acc_d = 0ul;
      this->acc_length = 0ul;
      this->acc_heading = 0ul;
      this->gnss_fix_ok = false;
      this->diff_soln = false;
      this->rel_pos_valid = false;
      this->is_moving = false;
      this->ref_pos_miss = false;
      this->ref_obs_miss = false;
      this->rel_pos_heading_valid = false;
      this->rel_pos_normalized = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _version_type =
    uint8_t;
  _version_type version;
  using _ref_station_id_type =
    uint16_t;
  _ref_station_id_type ref_station_id;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _rel_pos_n_type =
    int32_t;
  _rel_pos_n_type rel_pos_n;
  using _rel_pos_e_type =
    int32_t;
  _rel_pos_e_type rel_pos_e;
  using _rel_pos_d_type =
    int32_t;
  _rel_pos_d_type rel_pos_d;
  using _rel_pos_length_type =
    int32_t;
  _rel_pos_length_type rel_pos_length;
  using _rel_pos_heading_type =
    int32_t;
  _rel_pos_heading_type rel_pos_heading;
  using _rel_pos_hp_n_type =
    int8_t;
  _rel_pos_hp_n_type rel_pos_hp_n;
  using _rel_pos_hp_e_type =
    int8_t;
  _rel_pos_hp_e_type rel_pos_hp_e;
  using _rel_pos_hp_d_type =
    int8_t;
  _rel_pos_hp_d_type rel_pos_hp_d;
  using _rel_pos_hp_length_type =
    int8_t;
  _rel_pos_hp_length_type rel_pos_hp_length;
  using _acc_n_type =
    uint32_t;
  _acc_n_type acc_n;
  using _acc_e_type =
    uint32_t;
  _acc_e_type acc_e;
  using _acc_d_type =
    uint32_t;
  _acc_d_type acc_d;
  using _acc_length_type =
    uint32_t;
  _acc_length_type acc_length;
  using _acc_heading_type =
    uint32_t;
  _acc_heading_type acc_heading;
  using _gnss_fix_ok_type =
    bool;
  _gnss_fix_ok_type gnss_fix_ok;
  using _diff_soln_type =
    bool;
  _diff_soln_type diff_soln;
  using _rel_pos_valid_type =
    bool;
  _rel_pos_valid_type rel_pos_valid;
  using _carr_soln_type =
    ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator>;
  _carr_soln_type carr_soln;
  using _is_moving_type =
    bool;
  _is_moving_type is_moving;
  using _ref_pos_miss_type =
    bool;
  _ref_pos_miss_type ref_pos_miss;
  using _ref_obs_miss_type =
    bool;
  _ref_obs_miss_type ref_obs_miss;
  using _rel_pos_heading_valid_type =
    bool;
  _rel_pos_heading_valid_type rel_pos_heading_valid;
  using _rel_pos_normalized_type =
    bool;
  _rel_pos_normalized_type rel_pos_normalized;

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
  Type & set__ref_station_id(
    const uint16_t & _arg)
  {
    this->ref_station_id = _arg;
    return *this;
  }
  Type & set__itow(
    const uint32_t & _arg)
  {
    this->itow = _arg;
    return *this;
  }
  Type & set__rel_pos_n(
    const int32_t & _arg)
  {
    this->rel_pos_n = _arg;
    return *this;
  }
  Type & set__rel_pos_e(
    const int32_t & _arg)
  {
    this->rel_pos_e = _arg;
    return *this;
  }
  Type & set__rel_pos_d(
    const int32_t & _arg)
  {
    this->rel_pos_d = _arg;
    return *this;
  }
  Type & set__rel_pos_length(
    const int32_t & _arg)
  {
    this->rel_pos_length = _arg;
    return *this;
  }
  Type & set__rel_pos_heading(
    const int32_t & _arg)
  {
    this->rel_pos_heading = _arg;
    return *this;
  }
  Type & set__rel_pos_hp_n(
    const int8_t & _arg)
  {
    this->rel_pos_hp_n = _arg;
    return *this;
  }
  Type & set__rel_pos_hp_e(
    const int8_t & _arg)
  {
    this->rel_pos_hp_e = _arg;
    return *this;
  }
  Type & set__rel_pos_hp_d(
    const int8_t & _arg)
  {
    this->rel_pos_hp_d = _arg;
    return *this;
  }
  Type & set__rel_pos_hp_length(
    const int8_t & _arg)
  {
    this->rel_pos_hp_length = _arg;
    return *this;
  }
  Type & set__acc_n(
    const uint32_t & _arg)
  {
    this->acc_n = _arg;
    return *this;
  }
  Type & set__acc_e(
    const uint32_t & _arg)
  {
    this->acc_e = _arg;
    return *this;
  }
  Type & set__acc_d(
    const uint32_t & _arg)
  {
    this->acc_d = _arg;
    return *this;
  }
  Type & set__acc_length(
    const uint32_t & _arg)
  {
    this->acc_length = _arg;
    return *this;
  }
  Type & set__acc_heading(
    const uint32_t & _arg)
  {
    this->acc_heading = _arg;
    return *this;
  }
  Type & set__gnss_fix_ok(
    const bool & _arg)
  {
    this->gnss_fix_ok = _arg;
    return *this;
  }
  Type & set__diff_soln(
    const bool & _arg)
  {
    this->diff_soln = _arg;
    return *this;
  }
  Type & set__rel_pos_valid(
    const bool & _arg)
  {
    this->rel_pos_valid = _arg;
    return *this;
  }
  Type & set__carr_soln(
    const ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator> & _arg)
  {
    this->carr_soln = _arg;
    return *this;
  }
  Type & set__is_moving(
    const bool & _arg)
  {
    this->is_moving = _arg;
    return *this;
  }
  Type & set__ref_pos_miss(
    const bool & _arg)
  {
    this->ref_pos_miss = _arg;
    return *this;
  }
  Type & set__ref_obs_miss(
    const bool & _arg)
  {
    this->ref_obs_miss = _arg;
    return *this;
  }
  Type & set__rel_pos_heading_valid(
    const bool & _arg)
  {
    this->rel_pos_heading_valid = _arg;
    return *this;
  }
  Type & set__rel_pos_normalized(
    const bool & _arg)
  {
    this->rel_pos_normalized = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavRelPosNED
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavRelPosNED
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavRelPosNED_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavRelPosNED_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->ref_station_id != other.ref_station_id) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->rel_pos_n != other.rel_pos_n) {
      return false;
    }
    if (this->rel_pos_e != other.rel_pos_e) {
      return false;
    }
    if (this->rel_pos_d != other.rel_pos_d) {
      return false;
    }
    if (this->rel_pos_length != other.rel_pos_length) {
      return false;
    }
    if (this->rel_pos_heading != other.rel_pos_heading) {
      return false;
    }
    if (this->rel_pos_hp_n != other.rel_pos_hp_n) {
      return false;
    }
    if (this->rel_pos_hp_e != other.rel_pos_hp_e) {
      return false;
    }
    if (this->rel_pos_hp_d != other.rel_pos_hp_d) {
      return false;
    }
    if (this->rel_pos_hp_length != other.rel_pos_hp_length) {
      return false;
    }
    if (this->acc_n != other.acc_n) {
      return false;
    }
    if (this->acc_e != other.acc_e) {
      return false;
    }
    if (this->acc_d != other.acc_d) {
      return false;
    }
    if (this->acc_length != other.acc_length) {
      return false;
    }
    if (this->acc_heading != other.acc_heading) {
      return false;
    }
    if (this->gnss_fix_ok != other.gnss_fix_ok) {
      return false;
    }
    if (this->diff_soln != other.diff_soln) {
      return false;
    }
    if (this->rel_pos_valid != other.rel_pos_valid) {
      return false;
    }
    if (this->carr_soln != other.carr_soln) {
      return false;
    }
    if (this->is_moving != other.is_moving) {
      return false;
    }
    if (this->ref_pos_miss != other.ref_pos_miss) {
      return false;
    }
    if (this->ref_obs_miss != other.ref_obs_miss) {
      return false;
    }
    if (this->rel_pos_heading_valid != other.rel_pos_heading_valid) {
      return false;
    }
    if (this->rel_pos_normalized != other.rel_pos_normalized) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavRelPosNED_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavRelPosNED_

// alias to use template instance with default allocator
using UBXNavRelPosNED =
  ublox_ubx_msgs::msg::UBXNavRelPosNED_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_REL_POS_NED__STRUCT_HPP_
