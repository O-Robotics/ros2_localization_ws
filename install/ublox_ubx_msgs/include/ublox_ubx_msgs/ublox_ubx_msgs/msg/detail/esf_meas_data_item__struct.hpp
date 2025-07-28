// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_msgs:msg/ESFMeasDataItem.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__STRUCT_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_msgs__msg__ESFMeasDataItem __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_msgs__msg__ESFMeasDataItem __declspec(deprecated)
#endif

namespace ublox_ubx_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ESFMeasDataItem_
{
  using Type = ESFMeasDataItem_<ContainerAllocator>;

  explicit ESFMeasDataItem_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_field = 0ul;
      this->data_type = 0;
    }
  }

  explicit ESFMeasDataItem_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_field = 0ul;
      this->data_type = 0;
    }
  }

  // field types and members
  using _data_field_type =
    uint32_t;
  _data_field_type data_field;
  using _data_type_type =
    uint8_t;
  _data_type_type data_type;

  // setters for named parameter idiom
  Type & set__data_field(
    const uint32_t & _arg)
  {
    this->data_field = _arg;
    return *this;
  }
  Type & set__data_type(
    const uint8_t & _arg)
  {
    this->data_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_msgs__msg__ESFMeasDataItem
    std::shared_ptr<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_msgs__msg__ESFMeasDataItem
    std::shared_ptr<ublox_ubx_msgs::msg::ESFMeasDataItem_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ESFMeasDataItem_ & other) const
  {
    if (this->data_field != other.data_field) {
      return false;
    }
    if (this->data_type != other.data_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const ESFMeasDataItem_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ESFMeasDataItem_

// alias to use template instance with default allocator
using ESFMeasDataItem =
  ublox_ubx_msgs::msg::ESFMeasDataItem_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__ESF_MEAS_DATA_ITEM__STRUCT_HPP_
