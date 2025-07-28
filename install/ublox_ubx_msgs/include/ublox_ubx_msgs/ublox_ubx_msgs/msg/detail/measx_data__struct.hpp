// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/MeasxData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__MeasxData __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__MeasxData __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeasxData_
{
  using Type = MeasxData_<ContainerAllocator>;

  explicit MeasxData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gnss_id = 0;
      this->sv_id = 0;
      this->c_no = 0;
      this->mpath_indic = 0;
      this->doppler_ms = 0l;
      this->doppler_hz = 0l;
      this->whole_chips = 0;
      this->frac_chips = 0;
      this->code_phase = 0ul;
      this->int_code_phase = 0;
      this->pseu_range_rms_err = 0;
    }
  }

  explicit MeasxData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gnss_id = 0;
      this->sv_id = 0;
      this->c_no = 0;
      this->mpath_indic = 0;
      this->doppler_ms = 0l;
      this->doppler_hz = 0l;
      this->whole_chips = 0;
      this->frac_chips = 0;
      this->code_phase = 0ul;
      this->int_code_phase = 0;
      this->pseu_range_rms_err = 0;
    }
  }

  // field types and members
  using _gnss_id_type =
    uint8_t;
  _gnss_id_type gnss_id;
  using _sv_id_type =
    uint8_t;
  _sv_id_type sv_id;
  using _c_no_type =
    uint8_t;
  _c_no_type c_no;
  using _mpath_indic_type =
    uint8_t;
  _mpath_indic_type mpath_indic;
  using _doppler_ms_type =
    int32_t;
  _doppler_ms_type doppler_ms;
  using _doppler_hz_type =
    int32_t;
  _doppler_hz_type doppler_hz;
  using _whole_chips_type =
    uint16_t;
  _whole_chips_type whole_chips;
  using _frac_chips_type =
    uint16_t;
  _frac_chips_type frac_chips;
  using _code_phase_type =
    uint32_t;
  _code_phase_type code_phase;
  using _int_code_phase_type =
    uint8_t;
  _int_code_phase_type int_code_phase;
  using _pseu_range_rms_err_type =
    uint8_t;
  _pseu_range_rms_err_type pseu_range_rms_err;

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
  Type & set__c_no(
    const uint8_t & _arg)
  {
    this->c_no = _arg;
    return *this;
  }
  Type & set__mpath_indic(
    const uint8_t & _arg)
  {
    this->mpath_indic = _arg;
    return *this;
  }
  Type & set__doppler_ms(
    const int32_t & _arg)
  {
    this->doppler_ms = _arg;
    return *this;
  }
  Type & set__doppler_hz(
    const int32_t & _arg)
  {
    this->doppler_hz = _arg;
    return *this;
  }
  Type & set__whole_chips(
    const uint16_t & _arg)
  {
    this->whole_chips = _arg;
    return *this;
  }
  Type & set__frac_chips(
    const uint16_t & _arg)
  {
    this->frac_chips = _arg;
    return *this;
  }
  Type & set__code_phase(
    const uint32_t & _arg)
  {
    this->code_phase = _arg;
    return *this;
  }
  Type & set__int_code_phase(
    const uint8_t & _arg)
  {
    this->int_code_phase = _arg;
    return *this;
  }
  Type & set__pseu_range_rms_err(
    const uint8_t & _arg)
  {
    this->pseu_range_rms_err = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MPATH_NOT_MEASURED =
    0u;
  static constexpr uint8_t MPATH_LOW =
    1u;
  static constexpr uint8_t MPATH_MEDIUM =
    2u;
  static constexpr uint8_t MPATH_HIGH =
    3u;

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__MeasxData
    std::shared_ptr<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__MeasxData
    std::shared_ptr<ublox_ubx_msgs::msg::MeasxData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeasxData_ & other) const
  {
    if (this->gnss_id != other.gnss_id) {
      return false;
    }
    if (this->sv_id != other.sv_id) {
      return false;
    }
    if (this->c_no != other.c_no) {
      return false;
    }
    if (this->mpath_indic != other.mpath_indic) {
      return false;
    }
    if (this->doppler_ms != other.doppler_ms) {
      return false;
    }
    if (this->doppler_hz != other.doppler_hz) {
      return false;
    }
    if (this->whole_chips != other.whole_chips) {
      return false;
    }
    if (this->frac_chips != other.frac_chips) {
      return false;
    }
    if (this->code_phase != other.code_phase) {
      return false;
    }
    if (this->int_code_phase != other.int_code_phase) {
      return false;
    }
    if (this->pseu_range_rms_err != other.pseu_range_rms_err) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeasxData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeasxData_

// alias to use template instance with default allocator
using MeasxData =
  ublox_ubx_msgs::msg::MeasxData_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MeasxData_<ContainerAllocator>::MPATH_NOT_MEASURED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MeasxData_<ContainerAllocator>::MPATH_LOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MeasxData_<ContainerAllocator>::MPATH_MEDIUM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MeasxData_<ContainerAllocator>::MPATH_HIGH;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__MEASX_DATA__STRUCT_HPP_
