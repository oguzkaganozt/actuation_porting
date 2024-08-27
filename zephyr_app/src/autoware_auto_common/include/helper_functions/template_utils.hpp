// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_common/include/helper_functions/template_utils.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef HELPER_FUNCTIONS__TEMPLATE_UTILS_HPP_
#define HELPER_FUNCTIONS__TEMPLATE_UTILS_HPP_

#include <common/types.hpp>
#include <type_traits>

namespace autoware
{
namespace common
{
namespace helper_functions
{
/// This struct is `std::true_type` if the expression is valid for a given template and
/// `std::false_type` otherwise.
/// \tparam ExpressionTemplate Expression to be checked in compile time
/// \tparam T Template parameter to instantiate the expression.
template<template<typename ...> class ExpressionTemplate, typename T, typename = void>
struct expression_valid : std::false_type {};

/// This struct is `std::true_type` if the expression is valid for a given template and
/// `std::false_type` otherwise.
/// \tparam ExpressionTemplate Expression to be checked in compile time
/// \tparam T Template parameter to instantiate the expression.
template<template<typename ...> class ExpressionTemplate, typename T>
struct expression_valid<ExpressionTemplate, T,
  types::void_t<ExpressionTemplate<T>>>: std::true_type {};

/// This struct is `std::true_type` if the expression is valid for a given template
/// type with the specified return type and `std::false_type` otherwise.
/// \tparam ExpressionTemplate Expression to be checked in compile time
/// \tparam T Template parameter to instantiate the expression.
/// \tparam ReturnT Return type of the expression.
template<template<typename ...> class ExpressionTemplate, typename T, typename ReturnT,
  typename = void>
struct expression_valid_with_return : std::false_type {};

/// This struct is `std::true_type` if the expression is valid for a given template
/// type with the specified return type and `std::false_type` otherwise.
/// \tparam ExpressionTemplate Expression to be checked in compile time
/// \tparam T Template parameter to instantiate the expression.
/// \tparam ReturnT Return type of the expression.
template<template<typename ...> class ExpressionTemplate, typename T, typename ReturnT>
struct expression_valid_with_return<ExpressionTemplate, T, ReturnT,
  std::enable_if_t<std::is_same<ReturnT,
  ExpressionTemplate<T>>::value>>: std::true_type {};


}  // namespace helper_functions
}  // namespace common
}  // namespace autoware
#endif  // HELPER_FUNCTIONS__TEMPLATE_UTILS_HPP_
