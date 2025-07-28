// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavStatus.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__STRUCT_HPP_

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
// Member 'gps_fix'
#include "ublox_ubx_msgs/msg/detail/gps_fix__struct.hpp"
// Member 'map_matching'
#include "ublox_ubx_msgs/msg/detail/map_matching__struct.hpp"
// Member 'psm'
#include "ublox_ubx_msgs/msg/detail/psm_status__struct.hpp"
// Member 'spoof_det'
#include "ublox_ubx_msgs/msg/detail/spoof_det__struct.hpp"
// Member 'carr_soln'
#include "ublox_ubx_msgs/msg/detail/carr_soln__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavStatus __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXNavStatus __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXNavStatus_
{
  using Type = UBXNavStatus_<ContainerAllocator>;

  explicit UBXNavStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    gps_fix(_init),
    map_matching(_init),
    psm(_init),
    spoof_det(_init),
    carr_soln(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->gps_fix_ok = false;
      this->diff_soln = false;
      this->wkn_set = false;
      this->tow_set = false;
      this->diff_corr = false;
      this->carr_soln_valid = false;
      this->ttff = 0ul;
      this->msss = 0ul;
    }
  }

  explicit UBXNavStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    gps_fix(_alloc, _init),
    map_matching(_alloc, _init),
    psm(_alloc, _init),
    spoof_det(_alloc, _init),
    carr_soln(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->gps_fix_ok = false;
      this->diff_soln = false;
      this->wkn_set = false;
      this->tow_set = false;
      this->diff_corr = false;
      this->carr_soln_valid = false;
      this->ttff = 0ul;
      this->msss = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _gps_fix_type =
    ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator>;
  _gps_fix_type gps_fix;
  using _gps_fix_ok_type =
    bool;
  _gps_fix_ok_type gps_fix_ok;
  using _diff_soln_type =
    bool;
  _diff_soln_type diff_soln;
  using _wkn_set_type =
    bool;
  _wkn_set_type wkn_set;
  using _tow_set_type =
    bool;
  _tow_set_type tow_set;
  using _diff_corr_type =
    bool;
  _diff_corr_type diff_corr;
  using _carr_soln_valid_type =
    bool;
  _carr_soln_valid_type carr_soln_valid;
  using _map_matching_type =
    ublox_ubx_msgs::msg::MapMatching_<ContainerAllocator>;
  _map_matching_type map_matching;
  using _psm_type =
    ublox_ubx_msgs::msg::PSMStatus_<ContainerAllocator>;
  _psm_type psm;
  using _spoof_det_type =
    ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator>;
  _spoof_det_type spoof_det;
  using _carr_soln_type =
    ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator>;
  _carr_soln_type carr_soln;
  using _ttff_type =
    uint32_t;
  _ttff_type ttff;
  using _msss_type =
    uint32_t;
  _msss_type msss;

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
  Type & set__gps_fix(
    const ublox_ubx_msgs::msg::GpsFix_<ContainerAllocator> & _arg)
  {
    this->gps_fix = _arg;
    return *this;
  }
  Type & set__gps_fix_ok(
    const bool & _arg)
  {
    this->gps_fix_ok = _arg;
    return *this;
  }
  Type & set__diff_soln(
    const bool & _arg)
  {
    this->diff_soln = _arg;
    return *this;
  }
  Type & set__wkn_set(
    const bool & _arg)
  {
    this->wkn_set = _arg;
    return *this;
  }
  Type & set__tow_set(
    const bool & _arg)
  {
    this->tow_set = _arg;
    return *this;
  }
  Type & set__diff_corr(
    const bool & _arg)
  {
    this->diff_corr = _arg;
    return *this;
  }
  Type & set__carr_soln_valid(
    const bool & _arg)
  {
    this->carr_soln_valid = _arg;
    return *this;
  }
  Type & set__map_matching(
    const ublox_ubx_msgs::msg::MapMatching_<ContainerAllocator> & _arg)
  {
    this->map_matching = _arg;
    return *this;
  }
  Type & set__psm(
    const ublox_ubx_msgs::msg::PSMStatus_<ContainerAllocator> & _arg)
  {
    this->psm = _arg;
    return *this;
  }
  Type & set__spoof_det(
    const ublox_ubx_msgs::msg::SpoofDet_<ContainerAllocator> & _arg)
  {
    this->spoof_det = _arg;
    return *this;
  }
  Type & set__carr_soln(
    const ublox_ubx_msgs::msg::CarrSoln_<ContainerAllocator> & _arg)
  {
    this->carr_soln = _arg;
    return *this;
  }
  Type & set__ttff(
    const uint32_t & _arg)
  {
    this->ttff = _arg;
    return *this;
  }
  Type & set__msss(
    const uint32_t & _arg)
  {
    this->msss = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavStatus
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXNavStatus
    std::shared_ptr<ublox_ubx_msgs::msg::UBXNavStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXNavStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->gps_fix != other.gps_fix) {
      return false;
    }
    if (this->gps_fix_ok != other.gps_fix_ok) {
      return false;
    }
    if (this->diff_soln != other.diff_soln) {
      return false;
    }
    if (this->wkn_set != other.wkn_set) {
      return false;
    }
    if (this->tow_set != other.tow_set) {
      return false;
    }
    if (this->diff_corr != other.diff_corr) {
      return false;
    }
    if (this->carr_soln_valid != other.carr_soln_valid) {
      return false;
    }
    if (this->map_matching != other.map_matching) {
      return false;
    }
    if (this->psm != other.psm) {
      return false;
    }
    if (this->spoof_det != other.spoof_det) {
      return false;
    }
    if (this->carr_soln != other.carr_soln) {
      return false;
    }
    if (this->ttff != other.ttff) {
      return false;
    }
    if (this->msss != other.msss) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXNavStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXNavStatus_

// alias to use template instance with default allocator
using UBXNavStatus =
  ublox_ubx_msgs::msg::UBXNavStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_STATUS__STRUCT_HPP_
