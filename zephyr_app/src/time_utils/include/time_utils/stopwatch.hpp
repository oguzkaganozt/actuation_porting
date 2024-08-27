// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/time_utils/include/time_utils/stopwatch.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019-2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TIME_UTILS__STOPWATCH_HPP_
#define TIME_UTILS__STOPWATCH_HPP_

#include <time_utils/visibility_control.hpp>

#include <chrono>
#include <type_traits>

namespace autoware
{
namespace common
{
namespace time_utils
{

namespace detail
{

template<typename T>
struct is_duration : public std::false_type {};

template<typename ScalarT, typename DurationT>
struct is_duration<std::chrono::duration<ScalarT, DurationT>>: public std::true_type {};

}  // namespace detail

///
/// @brief      This class describes a timer used to quickly measure elapsed time.
///
class TIME_UTILS_PUBLIC Stopwatch
{
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;

public:
  /// Creating a timer automatically starts it.
  Stopwatch() = default;

  /// Restart a running timer.
  inline void restart() noexcept {m_start = Clock::now();}

  ///
  /// @brief      Get the elapsed time in the specified duration.
  ///
  /// @tparam     UnitsT  A type that must be a specialization of std::chrono::duration<T, S>.
  ///
  /// @return     Time elapsed since the start (or a restart) of the timer represented in the
  ///             provided duration format.
  ///
  template<typename UnitsT>
  inline UnitsT measure() const noexcept
  {
    static_assert(
      detail::is_duration<UnitsT>::value,
      "The provided type must be an std::chrono::duration<...> type, "
      "e.g. std::chrono::milliseconds or similar.");
    return std::chrono::duration_cast<UnitsT>(Clock::now() - m_start);
  }

private:
  TimePoint m_start{Clock::now()};
};

}  // namespace time_utils
}  // namespace common
}  // namespace autoware

#endif  // TIME_UTILS__STOPWATCH_HPP_
