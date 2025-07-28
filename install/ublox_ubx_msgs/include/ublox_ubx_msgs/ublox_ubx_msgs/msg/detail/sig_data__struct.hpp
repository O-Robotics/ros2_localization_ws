// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/SigData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'sig_flags'
#include "ublox_ubx_msgs/msg/detail/sig_flags__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__SigData __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__SigData __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SigData_
{
  using Type = SigData_<ContainerAllocator>;

  explicit SigData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sig_flags(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gnss_id = 0;
      this->sv_id = 0;
      this->sig_id = 0;
      this->freq_id = 0;
      this->pr_res = 0;
      this->cno = 0;
      this->quality_ind = 0;
      this->corr_source = 0;
      this->iono_model = 0;
    }
  }

  explicit SigData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sig_flags(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gnss_id = 0;
      this->sv_id = 0;
      this->sig_id = 0;
      this->freq_id = 0;
      this->pr_res = 0;
      this->cno = 0;
      this->quality_ind = 0;
      this->corr_source = 0;
      this->iono_model = 0;
    }
  }

  // field types and members
  using _gnss_id_type =
    uint8_t;
  _gnss_id_type gnss_id;
  using _sv_id_type =
    uint8_t;
  _sv_id_type sv_id;
  using _sig_id_type =
    uint8_t;
  _sig_id_type sig_id;
  using _freq_id_type =
    uint8_t;
  _freq_id_type freq_id;
  using _pr_res_type =
    int16_t;
  _pr_res_type pr_res;
  using _cno_type =
    uint8_t;
  _cno_type cno;
  using _quality_ind_type =
    uint8_t;
  _quality_ind_type quality_ind;
  using _corr_source_type =
    uint8_t;
  _corr_source_type corr_source;
  using _iono_model_type =
    uint8_t;
  _iono_model_type iono_model;
  using _sig_flags_type =
    ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator>;
  _sig_flags_type sig_flags;

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
  Type & set__sig_id(
    const uint8_t & _arg)
  {
    this->sig_id = _arg;
    return *this;
  }
  Type & set__freq_id(
    const uint8_t & _arg)
  {
    this->freq_id = _arg;
    return *this;
  }
  Type & set__pr_res(
    const int16_t & _arg)
  {
    this->pr_res = _arg;
    return *this;
  }
  Type & set__cno(
    const uint8_t & _arg)
  {
    this->cno = _arg;
    return *this;
  }
  Type & set__quality_ind(
    const uint8_t & _arg)
  {
    this->quality_ind = _arg;
    return *this;
  }
  Type & set__corr_source(
    const uint8_t & _arg)
  {
    this->corr_source = _arg;
    return *this;
  }
  Type & set__iono_model(
    const uint8_t & _arg)
  {
    this->iono_model = _arg;
    return *this;
  }
  Type & set__sig_flags(
    const ublox_ubx_msgs::msg::SigFlags_<ContainerAllocator> & _arg)
  {
    this->sig_flags = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t QUALITY_NO_SIGNAL =
    0u;
  static constexpr uint8_t QUALITY_SEARCHING_SIGNAL =
    1u;
  static constexpr uint8_t QUALITY_SIGNAL_ACQUIRED =
    2u;
  static constexpr uint8_t QUALITY_SIGNAL_UNUSABLE =
    3u;
  static constexpr uint8_t QUALITY_CODE_LOCKED =
    4u;
  static constexpr uint8_t QUALITY_CODE_CARRIER_LOCKED =
    5u;
  static constexpr uint8_t CORR_NONE =
    0u;
  static constexpr uint8_t CORR_SBAS =
    1u;
  static constexpr uint8_t CORR_BEIDOU =
    2u;
  static constexpr uint8_t CORR_RTCM2 =
    3u;
  static constexpr uint8_t CORR_RTCM3_OSR =
    4u;
  static constexpr uint8_t CORR_RTCM3_SSR =
    5u;
  static constexpr uint8_t CORR_QZSS_SLAS =
    6u;
  static constexpr uint8_t CORR_SPARTN =
    7u;
  static constexpr uint8_t CORR_CLAS =
    8u;
  static constexpr uint8_t IONO_NONE =
    0u;
  static constexpr uint8_t IONO_KLOB_GPS =
    1u;
  static constexpr uint8_t IONO_SBAS =
    2u;
  static constexpr uint8_t IONO_KLOB_BEIDOU =
    3u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::SigData_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::SigData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SigData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::SigData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SigData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SigData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::SigData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::SigData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SigData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::SigData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__SigData
    std::shared_ptr<ublox_ubx_msgs::msg::SigData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__SigData
    std::shared_ptr<ublox_ubx_msgs::msg::SigData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SigData_ & other) const
  {
    if (this->gnss_id != other.gnss_id) {
      return false;
    }
    if (this->sv_id != other.sv_id) {
      return false;
    }
    if (this->sig_id != other.sig_id) {
      return false;
    }
    if (this->freq_id != other.freq_id) {
      return false;
    }
    if (this->pr_res != other.pr_res) {
      return false;
    }
    if (this->cno != other.cno) {
      return false;
    }
    if (this->quality_ind != other.quality_ind) {
      return false;
    }
    if (this->corr_source != other.corr_source) {
      return false;
    }
    if (this->iono_model != other.iono_model) {
      return false;
    }
    if (this->sig_flags != other.sig_flags) {
      return false;
    }
    return true;
  }
  bool operator!=(const SigData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SigData_

// alias to use template instance with default allocator
using SigData =
  ublox_ubx_msgs::msg::SigData_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::QUALITY_NO_SIGNAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::QUALITY_SEARCHING_SIGNAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::QUALITY_SIGNAL_ACQUIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::QUALITY_SIGNAL_UNUSABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::QUALITY_CODE_LOCKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::QUALITY_CODE_CARRIER_LOCKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::CORR_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::CORR_SBAS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::CORR_BEIDOU;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::CORR_RTCM2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::CORR_RTCM3_OSR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::CORR_RTCM3_SSR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::CORR_QZSS_SLAS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::CORR_SPARTN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::CORR_CLAS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::IONO_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::IONO_KLOB_GPS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::IONO_SBAS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SigData_<ContainerAllocator>::IONO_KLOB_BEIDOU;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__SIG_DATA__STRUCT_HPP_
