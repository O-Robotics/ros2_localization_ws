// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ublox_ubx_interfaces:srv/ColdStart.idl
// generated code does not contain a copyright notice

#ifndef UBLOX_UBX_INTERFACES__SRV__DETAIL__COLD_START__TRAITS_HPP_
#define UBLOX_UBX_INTERFACES__SRV__DETAIL__COLD_START__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ublox_ubx_interfaces/srv/detail/cold_start__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ublox_ubx_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ColdStart_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: reset_type
  {
    out << "reset_type: ";
    rosidl_generator_traits::value_to_yaml(msg.reset_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ColdStart_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: reset_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_type: ";
    rosidl_generator_traits::value_to_yaml(msg.reset_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ColdStart_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ublox_ubx_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ublox_ubx_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ublox_ubx_interfaces::srv::ColdStart_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_interfaces::srv::ColdStart_Request & msg)
{
  return ublox_ubx_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_interfaces::srv::ColdStart_Request>()
{
  return "ublox_ubx_interfaces::srv::ColdStart_Request";
}

template<>
inline const char * name<ublox_ubx_interfaces::srv::ColdStart_Request>()
{
  return "ublox_ubx_interfaces/srv/ColdStart_Request";
}

template<>
struct has_fixed_size<ublox_ubx_interfaces::srv::ColdStart_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_interfaces::srv::ColdStart_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_interfaces::srv::ColdStart_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ublox_ubx_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ColdStart_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ColdStart_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ColdStart_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ublox_ubx_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ublox_ubx_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ublox_ubx_interfaces::srv::ColdStart_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ublox_ubx_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ublox_ubx_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ublox_ubx_interfaces::srv::ColdStart_Response & msg)
{
  return ublox_ubx_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ublox_ubx_interfaces::srv::ColdStart_Response>()
{
  return "ublox_ubx_interfaces::srv::ColdStart_Response";
}

template<>
inline const char * name<ublox_ubx_interfaces::srv::ColdStart_Response>()
{
  return "ublox_ubx_interfaces/srv/ColdStart_Response";
}

template<>
struct has_fixed_size<ublox_ubx_interfaces::srv::ColdStart_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ublox_ubx_interfaces::srv::ColdStart_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ublox_ubx_interfaces::srv::ColdStart_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ublox_ubx_interfaces::srv::ColdStart>()
{
  return "ublox_ubx_interfaces::srv::ColdStart";
}

template<>
inline const char * name<ublox_ubx_interfaces::srv::ColdStart>()
{
  return "ublox_ubx_interfaces/srv/ColdStart";
}

template<>
struct has_fixed_size<ublox_ubx_interfaces::srv::ColdStart>
  : std::integral_constant<
    bool,
    has_fixed_size<ublox_ubx_interfaces::srv::ColdStart_Request>::value &&
    has_fixed_size<ublox_ubx_interfaces::srv::ColdStart_Response>::value
  >
{
};

template<>
struct has_bounded_size<ublox_ubx_interfaces::srv::ColdStart>
  : std::integral_constant<
    bool,
    has_bounded_size<ublox_ubx_interfaces::srv::ColdStart_Request>::value &&
    has_bounded_size<ublox_ubx_interfaces::srv::ColdStart_Response>::value
  >
{
};

template<>
struct is_service<ublox_ubx_interfaces::srv::ColdStart>
  : std::true_type
{
};

template<>
struct is_service_request<ublox_ubx_interfaces::srv::ColdStart_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ublox_ubx_interfaces::srv::ColdStart_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // UBLOX_UBX_INTERFACES__SRV__DETAIL__COLD_START__TRAITS_HPP_
