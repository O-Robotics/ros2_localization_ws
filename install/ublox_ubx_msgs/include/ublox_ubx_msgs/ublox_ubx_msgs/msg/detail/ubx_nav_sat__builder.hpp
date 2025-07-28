// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/UBXNavSat.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SAT__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/ubx_nav_sat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_UBXNavSat_sv_info
{
public:
  explicit Init_UBXNavSat_sv_info(::ublox_ubx_msgs::msg::UBXNavSat & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::UBXNavSat sv_info(::ublox_ubx_msgs::msg::UBXNavSat::_sv_info_type arg)
  {
    msg_.sv_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSat msg_;
};

class Init_UBXNavSat_num_svs
{
public:
  explicit Init_UBXNavSat_num_svs(::ublox_ubx_msgs::msg::UBXNavSat & msg)
  : msg_(msg)
  {}
  Init_UBXNavSat_sv_info num_svs(::ublox_ubx_msgs::msg::UBXNavSat::_num_svs_type arg)
  {
    msg_.num_svs = std::move(arg);
    return Init_UBXNavSat_sv_info(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSat msg_;
};

class Init_UBXNavSat_version
{
public:
  explicit Init_UBXNavSat_version(::ublox_ubx_msgs::msg::UBXNavSat & msg)
  : msg_(msg)
  {}
  Init_UBXNavSat_num_svs version(::ublox_ubx_msgs::msg::UBXNavSat::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UBXNavSat_num_svs(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSat msg_;
};

class Init_UBXNavSat_itow
{
public:
  explicit Init_UBXNavSat_itow(::ublox_ubx_msgs::msg::UBXNavSat & msg)
  : msg_(msg)
  {}
  Init_UBXNavSat_version itow(::ublox_ubx_msgs::msg::UBXNavSat::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_UBXNavSat_version(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSat msg_;
};

class Init_UBXNavSat_header
{
public:
  Init_UBXNavSat_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UBXNavSat_itow header(::ublox_ubx_msgs::msg::UBXNavSat::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UBXNavSat_itow(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::UBXNavSat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::UBXNavSat>()
{
  return ublox_ubx_msgs::msg::builder::Init_UBXNavSat_header();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__UBX_NAV_SAT__BUILDER_HPP_
