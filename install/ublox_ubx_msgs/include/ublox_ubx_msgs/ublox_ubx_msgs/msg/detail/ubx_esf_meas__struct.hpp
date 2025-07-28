// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXEsfMeas.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_MEAS__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_MEAS__STRUCT_HPP_

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
// Member 'data'
#include "ublox_ubx_msgs/msg/detail/esf_meas_data_item__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXEsfMeas __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXEsfMeas __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXEsfMeas_
{
  using Type = UBXEsfMeas_<ContainerAllocator>;

  explicit UBXEsfMeas_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_tag = 0ul;
      this->time_mark_sent = 0;
      this->time_mark_edge = false;
      this->calib_ttag_valid = false;
      this->num_meas = 0;
      this->id = 0;
      this->calib_ttag = 0ul;
    }
  }

  explicit UBXEsfMeas_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_tag = 0ul;
      this->time_mark_sent = 0;
      this->time_mark_edge = false;
      this->calib_ttag_valid = false;
      this->num_meas = 0;
      this->id = 0;
      this->calib_ttag = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_tag_type =
    uint32_t;
  _time_tag_type time_tag;
  using _time_mark_sent_type =
    uint8_t;
  _time_mark_sent_type time_mark_sent;
  using _time_mark_edge_type =
    bool;
  _time_mark_edge_type time_mark_edge;
  using _calib_ttag_valid_type =
    bool;
  _calib_ttag_valid_type calib_ttag_valid;
  using _num_meas_type =
    uint8_t;
  _num_meas_type num_meas;
  using _id_type =
    uint16_t;
  _id_type id;
  using _data_type =
    std::vector<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>>>;
  _data_type data;
  using _calib_ttag_type =
    uint32_t;
  _calib_ttag_type calib_ttag;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__time_tag(
    const uint32_t & _arg)
  {
    this->time_tag = _arg;
    return *this;
  }
  Type & set__time_mark_sent(
    const uint8_t & _arg)
  {
    this->time_mark_sent = _arg;
    return *this;
  }
  Type & set__time_mark_edge(
    const bool & _arg)
  {
    this->time_mark_edge = _arg;
    return *this;
  }
  Type & set__calib_ttag_valid(
    const bool & _arg)
  {
    this->calib_ttag_valid = _arg;
    return *this;
  }
  Type & set__num_meas(
    const uint8_t & _arg)
  {
    this->num_meas = _arg;
    return *this;
  }
  Type & set__id(
    const uint16_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__calib_ttag(
    const uint32_t & _arg)
  {
    this->calib_ttag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXEsfMeas
    std::shared_ptr<ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXEsfMeas
    std::shared_ptr<ublox_ubx_msgs::msg::UBXEsfMeas_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXEsfMeas_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time_tag != other.time_tag) {
      return false;
    }
    if (this->time_mark_sent != other.time_mark_sent) {
      return false;
    }
    if (this->time_mark_edge != other.time_mark_edge) {
      return false;
    }
    if (this->calib_ttag_valid != other.calib_ttag_valid) {
      return false;
    }
    if (this->num_meas != other.num_meas) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->calib_ttag != other.calib_ttag) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXEsfMeas_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXEsfMeas_

// alias to use template instance with default allocator
using UBXEsfMeas =
  ublox_ubx_msgs::msg::UBXEsfMeas_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_ESF_MEAS__STRUCT_HPP_
