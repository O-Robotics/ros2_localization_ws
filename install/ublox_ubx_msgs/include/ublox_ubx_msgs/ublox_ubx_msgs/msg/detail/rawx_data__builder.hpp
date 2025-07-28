// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ublox_ubx_msgs:msg/RawxData.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__BUILDER_HPP_
#define UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ublox_ubx_msgs/msg/detail/rawx_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ublox_ubx_msgs
{

namespace msg
{

namespace builder
{

class Init_RawxData_trk_stat
{
public:
  explicit Init_RawxData_trk_stat(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  ::ublox_ubx_msgs::msg::RawxData trk_stat(::ublox_ubx_msgs::msg::RawxData::_trk_stat_type arg)
  {
    msg_.trk_stat = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_do_stdev
{
public:
  explicit Init_RawxData_do_stdev(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_trk_stat do_stdev(::ublox_ubx_msgs::msg::RawxData::_do_stdev_type arg)
  {
    msg_.do_stdev = std::move(arg);
    return Init_RawxData_trk_stat(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_cp_stdev
{
public:
  explicit Init_RawxData_cp_stdev(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_do_stdev cp_stdev(::ublox_ubx_msgs::msg::RawxData::_cp_stdev_type arg)
  {
    msg_.cp_stdev = std::move(arg);
    return Init_RawxData_do_stdev(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_pr_stdev
{
public:
  explicit Init_RawxData_pr_stdev(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_cp_stdev pr_stdev(::ublox_ubx_msgs::msg::RawxData::_pr_stdev_type arg)
  {
    msg_.pr_stdev = std::move(arg);
    return Init_RawxData_cp_stdev(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_c_no
{
public:
  explicit Init_RawxData_c_no(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_pr_stdev c_no(::ublox_ubx_msgs::msg::RawxData::_c_no_type arg)
  {
    msg_.c_no = std::move(arg);
    return Init_RawxData_pr_stdev(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_locktime
{
public:
  explicit Init_RawxData_locktime(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_c_no locktime(::ublox_ubx_msgs::msg::RawxData::_locktime_type arg)
  {
    msg_.locktime = std::move(arg);
    return Init_RawxData_c_no(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_freq_id
{
public:
  explicit Init_RawxData_freq_id(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_locktime freq_id(::ublox_ubx_msgs::msg::RawxData::_freq_id_type arg)
  {
    msg_.freq_id = std::move(arg);
    return Init_RawxData_locktime(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_sig_id
{
public:
  explicit Init_RawxData_sig_id(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_freq_id sig_id(::ublox_ubx_msgs::msg::RawxData::_sig_id_type arg)
  {
    msg_.sig_id = std::move(arg);
    return Init_RawxData_freq_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_sv_id
{
public:
  explicit Init_RawxData_sv_id(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_sig_id sv_id(::ublox_ubx_msgs::msg::RawxData::_sv_id_type arg)
  {
    msg_.sv_id = std::move(arg);
    return Init_RawxData_sig_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_gnss_id
{
public:
  explicit Init_RawxData_gnss_id(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_sv_id gnss_id(::ublox_ubx_msgs::msg::RawxData::_gnss_id_type arg)
  {
    msg_.gnss_id = std::move(arg);
    return Init_RawxData_sv_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_do_mes
{
public:
  explicit Init_RawxData_do_mes(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_gnss_id do_mes(::ublox_ubx_msgs::msg::RawxData::_do_mes_type arg)
  {
    msg_.do_mes = std::move(arg);
    return Init_RawxData_gnss_id(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_cp_mes
{
public:
  explicit Init_RawxData_cp_mes(::ublox_ubx_msgs::msg::RawxData & msg)
  : msg_(msg)
  {}
  Init_RawxData_do_mes cp_mes(::ublox_ubx_msgs::msg::RawxData::_cp_mes_type arg)
  {
    msg_.cp_mes = std::move(arg);
    return Init_RawxData_do_mes(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

class Init_RawxData_pr_mes
{
public:
  Init_RawxData_pr_mes()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RawxData_cp_mes pr_mes(::ublox_ubx_msgs::msg::RawxData::_pr_mes_type arg)
  {
    msg_.pr_mes = std::move(arg);
    return Init_RawxData_cp_mes(msg_);
  }

private:
  ::ublox_ubx_msgs::msg::RawxData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ublox_ubx_msgs::msg::RawxData>()
{
  return ublox_ubx_msgs::msg::builder::Init_RawxData_pr_mes();
}

}  // namespace ublox_ubx_msgs

#endif  // UBLOX_UBX_MSGS__MSG__DETAIL__RAWX_DATA__BUILDER_HPP_
