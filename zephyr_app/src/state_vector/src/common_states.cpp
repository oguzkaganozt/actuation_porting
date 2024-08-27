// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/state_vector/src/common_states.cpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// @file
/// \brief This file has a number of explicit instantiations that are used throughout the code base.

#include <state_vector/common_states.hpp>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace common
{
namespace state_vector
{

template class GenericState<float32_t,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION>;
template class GenericState<float64_t,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION>;


template class GenericState<float32_t,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
    variable::YAW, variable::YAW_CHANGE_RATE, variable::YAW_CHANGE_ACCELERATION>;
template class GenericState<float64_t,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
    variable::YAW, variable::YAW_CHANGE_RATE, variable::YAW_CHANGE_ACCELERATION>;

template class GenericState<float32_t,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE,
    variable::XY_ACCELERATION>;
template class GenericState<float64_t,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE,
    variable::XY_ACCELERATION>;

template class GenericState<float32_t,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE>;
template class GenericState<float64_t,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE>;

}  // namespace state_vector
}  // namespace common
}  // namespace autoware
