// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_common/include/helper_functions/bool_comparisons.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2020, Mapless AI, Inc.
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: MIT

#ifndef HELPER_FUNCTIONS__BOOL_COMPARISONS_HPP_
#define HELPER_FUNCTIONS__BOOL_COMPARISONS_HPP_

#include "common/types.hpp"

namespace autoware
{
namespace common
{
namespace helper_functions
{
namespace comparisons
{

/**
 * @brief Convenience method for performing logical exclusive or ops.
 * @return True iff exactly one of 'a' and 'b' is true.
 */
template<typename T>
types::bool8_t exclusive_or(const T & a, const T & b)
{
  return static_cast<types::bool8_t>(a) != static_cast<types::bool8_t>(b);
}

}  // namespace comparisons
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__BOOL_COMPARISONS_HPP_
