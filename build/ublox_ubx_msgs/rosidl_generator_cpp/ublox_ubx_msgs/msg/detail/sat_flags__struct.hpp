// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/SatFlags.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__SatFlags __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__SatFlags __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SatFlags_
{
  using Type = SatFlags_<ContainerAllocator>;

  explicit SatFlags_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quality_ind = 0;
      this->sv_used = false;
      this->health = 0;
      this->diff_corr = false;
      this->smoothed = false;
      this->orbit_source = 0;
      this->eph_avail = false;
      this->alm_avail = false;
      this->ano_avail = false;
      this->aop_avail = false;
      this->sbas_corr_used = false;
      this->rtcm_corr_used = false;
      this->slas_corr_used = false;
      this->spartn_corr_used = false;
      this->pr_corr_used = false;
      this->cr_corr_used = false;
      this->do_corr_used = false;
      this->clas_corr_used = false;
    }
  }

  explicit SatFlags_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quality_ind = 0;
      this->sv_used = false;
      this->health = 0;
      this->diff_corr = false;
      this->smoothed = false;
      this->orbit_source = 0;
      this->eph_avail = false;
      this->alm_avail = false;
      this->ano_avail = false;
      this->aop_avail = false;
      this->sbas_corr_used = false;
      this->rtcm_corr_used = false;
      this->slas_corr_used = false;
      this->spartn_corr_used = false;
      this->pr_corr_used = false;
      this->cr_corr_used = false;
      this->do_corr_used = false;
      this->clas_corr_used = false;
    }
  }

  // field types and members
  using _quality_ind_type =
    uint8_t;
  _quality_ind_type quality_ind;
  using _sv_used_type =
    bool;
  _sv_used_type sv_used;
  using _health_type =
    uint8_t;
  _health_type health;
  using _diff_corr_type =
    bool;
  _diff_corr_type diff_corr;
  using _smoothed_type =
    bool;
  _smoothed_type smoothed;
  using _orbit_source_type =
    uint8_t;
  _orbit_source_type orbit_source;
  using _eph_avail_type =
    bool;
  _eph_avail_type eph_avail;
  using _alm_avail_type =
    bool;
  _alm_avail_type alm_avail;
  using _ano_avail_type =
    bool;
  _ano_avail_type ano_avail;
  using _aop_avail_type =
    bool;
  _aop_avail_type aop_avail;
  using _sbas_corr_used_type =
    bool;
  _sbas_corr_used_type sbas_corr_used;
  using _rtcm_corr_used_type =
    bool;
  _rtcm_corr_used_type rtcm_corr_used;
  using _slas_corr_used_type =
    bool;
  _slas_corr_used_type slas_corr_used;
  using _spartn_corr_used_type =
    bool;
  _spartn_corr_used_type spartn_corr_used;
  using _pr_corr_used_type =
    bool;
  _pr_corr_used_type pr_corr_used;
  using _cr_corr_used_type =
    bool;
  _cr_corr_used_type cr_corr_used;
  using _do_corr_used_type =
    bool;
  _do_corr_used_type do_corr_used;
  using _clas_corr_used_type =
    bool;
  _clas_corr_used_type clas_corr_used;

  // setters for named parameter idiom
  Type & set__quality_ind(
    const uint8_t & _arg)
  {
    this->quality_ind = _arg;
    return *this;
  }
  Type & set__sv_used(
    const bool & _arg)
  {
    this->sv_used = _arg;
    return *this;
  }
  Type & set__health(
    const uint8_t & _arg)
  {
    this->health = _arg;
    return *this;
  }
  Type & set__diff_corr(
    const bool & _arg)
  {
    this->diff_corr = _arg;
    return *this;
  }
  Type & set__smoothed(
    const bool & _arg)
  {
    this->smoothed = _arg;
    return *this;
  }
  Type & set__orbit_source(
    const uint8_t & _arg)
  {
    this->orbit_source = _arg;
    return *this;
  }
  Type & set__eph_avail(
    const bool & _arg)
  {
    this->eph_avail = _arg;
    return *this;
  }
  Type & set__alm_avail(
    const bool & _arg)
  {
    this->alm_avail = _arg;
    return *this;
  }
  Type & set__ano_avail(
    const bool & _arg)
  {
    this->ano_avail = _arg;
    return *this;
  }
  Type & set__aop_avail(
    const bool & _arg)
  {
    this->aop_avail = _arg;
    return *this;
  }
  Type & set__sbas_corr_used(
    const bool & _arg)
  {
    this->sbas_corr_used = _arg;
    return *this;
  }
  Type & set__rtcm_corr_used(
    const bool & _arg)
  {
    this->rtcm_corr_used = _arg;
    return *this;
  }
  Type & set__slas_corr_used(
    const bool & _arg)
  {
    this->slas_corr_used = _arg;
    return *this;
  }
  Type & set__spartn_corr_used(
    const bool & _arg)
  {
    this->spartn_corr_used = _arg;
    return *this;
  }
  Type & set__pr_corr_used(
    const bool & _arg)
  {
    this->pr_corr_used = _arg;
    return *this;
  }
  Type & set__cr_corr_used(
    const bool & _arg)
  {
    this->cr_corr_used = _arg;
    return *this;
  }
  Type & set__do_corr_used(
    const bool & _arg)
  {
    this->do_corr_used = _arg;
    return *this;
  }
  Type & set__clas_corr_used(
    const bool & _arg)
  {
    this->clas_corr_used = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t QUALITY_NO_SIGNAL =
    0u;
  static constexpr uint8_t QUALITY_SEARCHING =
    1u;
  static constexpr uint8_t QUALITY_ACQUIRED =
    2u;
  static constexpr uint8_t QUALITY_UNUSABLE =
    3u;
  static constexpr uint8_t QUALITY_CODE_LOCKED =
    4u;
  static constexpr uint8_t QUALITY_CARRIER_LOCKED =
    5u;
  static constexpr uint8_t HEALTH_UNKNOWN =
    0u;
  static constexpr uint8_t HEALTH_HEALTHY =
    1u;
  static constexpr uint8_t HEALTH_UNHEALTHY =
    2u;
  static constexpr uint8_t ORBIT_NO_INFO =
    0u;
  static constexpr uint8_t ORBIT_EPH_USED =
    1u;
  static constexpr uint8_t ORBIT_ALM_USED =
    2u;
  static constexpr uint8_t ORBIT_ASSISTNOW_OFFLINE =
    3u;
  static constexpr uint8_t ORBIT_ASSISTNOW_AUTONOMOUS =
    4u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__SatFlags
    std::shared_ptr<ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__SatFlags
    std::shared_ptr<ublox_ubx_msgs::msg::SatFlags_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SatFlags_ & other) const
  {
    if (this->quality_ind != other.quality_ind) {
      return false;
    }
    if (this->sv_used != other.sv_used) {
      return false;
    }
    if (this->health != other.health) {
      return false;
    }
    if (this->diff_corr != other.diff_corr) {
      return false;
    }
    if (this->smoothed != other.smoothed) {
      return false;
    }
    if (this->orbit_source != other.orbit_source) {
      return false;
    }
    if (this->eph_avail != other.eph_avail) {
      return false;
    }
    if (this->alm_avail != other.alm_avail) {
      return false;
    }
    if (this->ano_avail != other.ano_avail) {
      return false;
    }
    if (this->aop_avail != other.aop_avail) {
      return false;
    }
    if (this->sbas_corr_used != other.sbas_corr_used) {
      return false;
    }
    if (this->rtcm_corr_used != other.rtcm_corr_used) {
      return false;
    }
    if (this->slas_corr_used != other.slas_corr_used) {
      return false;
    }
    if (this->spartn_corr_used != other.spartn_corr_used) {
      return false;
    }
    if (this->pr_corr_used != other.pr_corr_used) {
      return false;
    }
    if (this->cr_corr_used != other.cr_corr_used) {
      return false;
    }
    if (this->do_corr_used != other.do_corr_used) {
      return false;
    }
    if (this->clas_corr_used != other.clas_corr_used) {
      return false;
    }
    return true;
  }
  bool operator!=(const SatFlags_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SatFlags_

// alias to use template instance with default allocator
using SatFlags =
  ublox_ubx_msgs::msg::SatFlags_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::QUALITY_NO_SIGNAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::QUALITY_SEARCHING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::QUALITY_ACQUIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::QUALITY_UNUSABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::QUALITY_CODE_LOCKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::QUALITY_CARRIER_LOCKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::HEALTH_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::HEALTH_HEALTHY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::HEALTH_UNHEALTHY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::ORBIT_NO_INFO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::ORBIT_EPH_USED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::ORBIT_ALM_USED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::ORBIT_ASSISTNOW_OFFLINE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SatFlags_<ContainerAllocator>::ORBIT_ASSISTNOW_AUTONOMOUS;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SAT_FLAGS__STRUCT_HPP_
