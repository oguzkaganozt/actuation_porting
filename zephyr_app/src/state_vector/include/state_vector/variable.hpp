// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/state_vector/include/state_vector/variable.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief Contains base tag structs that define variables and traits to check if a type is one.

#ifndef STATE_VECTOR__VARIABLE_HPP_
#define STATE_VECTOR__VARIABLE_HPP_

#include <state_vector/visibility_control.hpp>

#include <type_traits>


namespace autoware
{
namespace common
{
namespace state_vector
{

///
/// @brief      A tag struct used to disambiguate variables from other types.
///
struct STATE_VECTOR_PUBLIC Variable {};

///
/// @brief      A tag struct used to disambiguate variables that store angles from other types.
///
///             Inheriting from AngleVariable allows to automatically wrap angles upon need.
///
struct STATE_VECTOR_PUBLIC AngleVariable : Variable {};

///
/// @brief      A trait to check if a type is a variable by checking if it inherits from Variable.
///
/// @tparam     T     Query type.
///
template<typename T>
struct STATE_VECTOR_PUBLIC is_variable : std::conditional_t<
    std::is_base_of<Variable, T>::value, std::true_type, std::false_type> {};

///
/// @brief      A trait to check if a variable represents an angle.
///
/// @tparam     T     Variable type.
///
template<typename T>
struct STATE_VECTOR_PUBLIC is_angle : std::conditional_t<
    std::is_base_of<AngleVariable, T>::value, std::true_type, std::false_type> {};


}  // namespace state_vector
}  // namespace common
}  // namespace autoware

#endif  // STATE_VECTOR__VARIABLE_HPP_
