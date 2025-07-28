// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ublox_ubx_interfaces:srv/WarmStart.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_INTERFACES__SRV__DETAIL__WARM_START__STRUCT_HPP_
#define UBLOX_UBX_INTERFACES__SRV__DETAIL__WARM_START__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_interfaces__srv__WarmStart_Request __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_interfaces__srv__WarmStart_Request __declspec(deprecated)
#endif

namespace ublox_ubx_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WarmStart_Request_
{
  using Type = WarmStart_Request_<ContainerAllocator>;

  explicit WarmStart_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reset_type = 0;
    }
  }

  explicit WarmStart_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reset_type = 0;
    }
  }

  // field types and members
  using _reset_type_type =
    uint8_t;
  _reset_type_type reset_type;

  // setters for named parameter idiom
  Type & set__reset_type(
    const uint8_t & _arg)
  {
    this->reset_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_interfaces__srv__WarmStart_Request
    std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_interfaces__srv__WarmStart_Request
    std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WarmStart_Request_ & other) const
  {
    if (this->reset_type != other.reset_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const WarmStart_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WarmStart_Request_

// alias to use template instance with default allocator
using WarmStart_Request =
  ublox_ubx_interfaces::srv::WarmStart_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ublox_ubx_interfaces


#ifndef _WIN32
# define DEPRECATED__ublox_ubx_interfaces__srv__WarmStart_Response __attribute__((deprecated))
#else
# define DEPRECATED__ublox_ubx_interfaces__srv__WarmStart_Response __declspec(deprecated)
#endif

namespace ublox_ubx_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WarmStart_Response_
{
  using Type = WarmStart_Response_<ContainerAllocator>;

  explicit WarmStart_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit WarmStart_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ublox_ubx_interfaces__srv__WarmStart_Response
    std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ublox_ubx_interfaces__srv__WarmStart_Response
    std::shared_ptr<ublox_ubx_interfaces::srv::WarmStart_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WarmStart_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const WarmStart_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WarmStart_Response_

// alias to use template instance with default allocator
using WarmStart_Response =
  ublox_ubx_interfaces::srv::WarmStart_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ublox_ubx_interfaces

namespace ublox_ubx_interfaces
{

namespace srv
{

struct WarmStart
{
  using Request = ublox_ubx_interfaces::srv::WarmStart_Request;
  using Response = ublox_ubx_interfaces::srv::WarmStart_Response;
};

}  // namespace srv

}  // namespace ublox_ubx_interfaces

#endif  // UBLOX_UBX_INTERFACES__SRV__DETAIL__WARM_START__STRUCT_HPP_
