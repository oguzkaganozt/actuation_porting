// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_common/include/helper_functions/message_adapters.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2017-2019, the Autoware Foundation
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief This file includes common helper functions

#ifndef HELPER_FUNCTIONS__MESSAGE_ADAPTERS_HPP_
#define HELPER_FUNCTIONS__MESSAGE_ADAPTERS_HPP_

#include <Time.h>
#include <string>

namespace autoware
{
namespace common
{
namespace helper_functions
{
namespace message_field_adapters
{
/// Using alias for Time message
using TimeStamp = builtin_interfaces_msg_Time;

/// \brief Helper class to check existance of header file in compile time:
/// https://stackoverflow.com/a/16000226/2325407
template<typename T, typename = nullptr_t>
struct HasHeader : std::false_type {};

template<typename T>
struct HasHeader<T, decltype((void) T::header, nullptr)>: std::true_type {};

/////////// Template declarations

/// Get frame id from message. nullptr_t is used to prevent template ambiguity on
/// SFINAE specializations. Provide a default value on specializations for a friendly API.
/// \tparam T Message type.
/// \param msg Message.
/// \return Frame id of the message.
template<typename T, nullptr_t>
const std::string & get_frame_id(const T & msg) noexcept;

/// Get a reference to the frame id from message. nullptr_t is used to prevent
/// template ambiguity on SFINAE specializations. Provide a default value on
/// specializations for a friendly API.
/// \tparam T Message type.
/// \param msg Message.
/// \return Frame id of the message.
template<typename T, nullptr_t>
std::string & get_frame_id(T & msg) noexcept;

/// Get stamp from message. nullptr_t is used to prevent template ambiguity on
/// SFINAE specializations. Provide a default value on specializations for a friendly API.
/// \tparam T Message type.
/// \param msg Message.
/// \return Frame id of the message.
template<typename T, nullptr_t>
const TimeStamp & get_stamp(const T & msg) noexcept;

/// Get a reference to the stamp from message. nullptr_t is used to prevent
/// template ambiguity on SFINAE specializations. Provide a default value on
/// specializations for a friendly API.
/// \tparam T Message type.
/// \param msg Message.
/// \return Frame id of the message.
template<typename T, nullptr_t>
TimeStamp & get_stamp(T & msg) noexcept;


/////////////// Default specializations for message types that contain a header.
template<class T, typename std::enable_if<HasHeader<T>::value, nullptr_t>::type = nullptr>
const std::string & get_frame_id(const T & msg) noexcept
{
  return msg.header.frame_id;
}

template<class T, typename std::enable_if<HasHeader<T>::value, nullptr_t>::type = nullptr>
std::string & get_frame_id(T & msg) noexcept
{
  return msg.header.frame_id;
}

template<class T, typename std::enable_if<HasHeader<T>::value, nullptr_t>::type = nullptr>
TimeStamp & get_stamp(T & msg) noexcept
{
  return msg.header.stamp;
}

template<class T, typename std::enable_if<HasHeader<T>::value, nullptr_t>::type = nullptr>
TimeStamp get_stamp(const T & msg) noexcept
{
  return msg.header.stamp;
}

}  // namespace message_field_adapters
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__MESSAGE_ADAPTERS_HPP_
