// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/state_vector/include/state_vector/common_variables.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file defines the common variables used within the filter implementations.

#ifndef STATE_VECTOR__COMMON_VARIABLES_HPP_
#define STATE_VECTOR__COMMON_VARIABLES_HPP_

#include <state_vector/variable.hpp>

#include <type_traits>

namespace autoware
{
namespace common
{
namespace state_vector
{

namespace variable
{
struct X : Variable {};
struct Y : Variable {};
struct Z : Variable {};

///
/// @brief      A variable that represents the roll angle of an object.
///
/// @note       While the value of this variable in a state vector itself can be set by the user
///             arbitrarily, throughout our code base we use the convention in which ROLL represents
///             the CCW rotation around the X axis.
///
struct ROLL : AngleVariable {};

///
/// @brief      A variable that represents the pitch angle of an object.
///
/// @note       While the value of this variable in a state vector itself can be set by the user
///             arbitrarily, throughout our code base we use the convention in which PITCH
///             represents the CCW rotation around the Y axis.
///
struct PITCH : AngleVariable {};

///
/// @brief      A variable that represents the roll angle of an object.
///
/// @note       While the value of this variable in a state vector itself can be set by the user
///             arbitrarily, throughout our code base we use the convention in which YAW
///             represents the CCW rotation around the Z axis.
///
struct YAW : AngleVariable {};

struct X_VELOCITY : Variable {};
struct Y_VELOCITY : Variable {};
struct Z_VELOCITY : Variable {};

struct ROLL_CHANGE_RATE : Variable {};
struct PITCH_CHANGE_RATE : Variable {};
struct YAW_CHANGE_RATE : Variable {};

struct X_ACCELERATION : Variable {};
struct Y_ACCELERATION : Variable {};
struct Z_ACCELERATION : Variable {};

struct ROLL_CHANGE_ACCELERATION : Variable {};
struct PITCH_CHANGE_ACCELERATION : Variable {};
struct YAW_CHANGE_ACCELERATION : Variable {};

/// Velocity in xy plane. Used in differential drive models.
struct XY_VELOCITY : Variable {};
/// Acceleration in xy plane. Used in differential drive models.
struct XY_ACCELERATION : Variable {};

}  // namespace variable

}  // namespace state_vector
}  // namespace common
}  // namespace autoware

#endif  // STATE_VECTOR__COMMON_VARIABLES_HPP_
