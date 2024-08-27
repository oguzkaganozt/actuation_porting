// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/motion_model/include/motion_model/motion_model_interface.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Developed by Apex.AI, Inc.

#ifndef MOTION_MODEL__MOTION_MODEL_INTERFACE_HPP_
#define MOTION_MODEL__MOTION_MODEL_INTERFACE_HPP_

#include <helper_functions/crtp.hpp>
#include <motion_model/visibility_control.hpp>
#include <state_vector/generic_state.hpp>

#include <chrono>

namespace autoware
{
namespace common
{
namespace motion_model
{

///
/// @brief      A CRTP interface for any motion model.
///
/// @tparam     Derived  Motion model implementation class.
///
template<typename Derived>
class MOTION_MODEL_PUBLIC MotionModelInterface : public common::helper_functions::crtp<Derived>
{
public:
  ///
  /// @brief      Get the next predicted state.
  ///
  /// @param[in]  state   The initial state
  /// @param[in]  dt      Length of prediction into the future
  ///
  /// @tparam     StateT  Type of the state.
  ///
  /// @return     Predicted state after the time has passed.
  ///
  template<typename StateT>
  inline auto predict(const StateT & state, const std::chrono::nanoseconds & dt) const
  {
    static_assert(
      common::state_vector::is_state<StateT>::value,
      "\n\nStateT must be a GenericState\n\n");
    return this->impl().crtp_predict(state, dt);
  }
  ///
  /// @brief      Get the Jacobian of this motion model.
  ///
  /// @param[in]  state   The current state.
  /// @param[in]  dt      Time span.
  ///
  /// @tparam     StateT  Type of the state.
  ///
  /// @return     A Jacobian of the motion model. In the linear case - a transition matrix.
  ///
  template<typename StateT>
  inline auto jacobian(const StateT & state, const std::chrono::nanoseconds & dt) const
  {
    static_assert(
      common::state_vector::is_state<StateT>::value,
      "\n\nStateT must be a GenericState\n\n");
    return this->impl().crtp_jacobian(state, dt);
  }
};

}  // namespace motion_model
}  // namespace common
}  // namespace autoware

#endif  // MOTION_MODEL__MOTION_MODEL_INTERFACE_HPP_
