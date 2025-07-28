// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/UBXSecSigLog.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG_LOG__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG_LOG__STRUCT_HPP_

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
// Member 'events'
#include "ublox_ubx_msgs/msg/detail/sig_log_event__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__UBXSecSigLog __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__UBXSecSigLog __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UBXSecSigLog_
{
  using Type = UBXSecSigLog_<ContainerAllocator>;

  explicit UBXSecSigLog_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->num_events = 0;
    }
  }

  explicit UBXSecSigLog_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version = 0;
      this->num_events = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _version_type =
    uint8_t;
  _version_type version;
  using _num_events_type =
    uint8_t;
  _num_events_type num_events;
  using _events_type =
    std::vector<ublox_ubx_msgs::msg::SigLogEvent_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::SigLogEvent_<ContainerAllocator>>>;
  _events_type events;

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
  Type & set__num_events(
    const uint8_t & _arg)
  {
    this->num_events = _arg;
    return *this;
  }
  Type & set__events(
    const std::vector<ublox_ubx_msgs::msg::SigLogEvent_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ublox_ubx_msgs::msg::SigLogEvent_<ContainerAllocator>>> & _arg)
  {
    this->events = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXSecSigLog
    std::shared_ptr<ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__UBXSecSigLog
    std::shared_ptr<ublox_ubx_msgs::msg::UBXSecSigLog_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UBXSecSigLog_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->num_events != other.num_events) {
      return false;
    }
    if (this->events != other.events) {
      return false;
    }
    return true;
  }
  bool operator!=(const UBXSecSigLog_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UBXSecSigLog_

// alias to use template instance with default allocator
using UBXSecSigLog =
  ublox_ubx_msgs::msg::UBXSecSigLog_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_SEC_SIG_LOG__STRUCT_HPP_
