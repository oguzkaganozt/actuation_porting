// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/motion_model/include/motion_model/linear_motion_model.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Developed by Apex.AI, Inc.

#ifndef MOTION_MODEL__LINEAR_MOTION_MODEL_HPP_
#define MOTION_MODEL__LINEAR_MOTION_MODEL_HPP_

#include <motion_model/motion_model_interface.hpp>
#include <motion_model/visibility_control.hpp>
#include <state_vector/common_states.hpp>
#include <state_vector/generic_state.hpp>

namespace autoware
{
namespace common
{
namespace motion_model
{

///
/// @brief      A generic linear motion model class.
///
/// @details    This class is designed to handle all the linear motion models, i.e., those that only
///             have independent variables in them. In this case, the Jacobian is just a transition
///             matrix.
///
/// @tparam     StateT  State type.
///
template<typename StateT>
class MOTION_MODEL_PUBLIC LinearMotionModel
  : public MotionModelInterface<LinearMotionModel<StateT>>
{
public:
  using State = StateT;

protected:
  // Allow the CRTP interface to call private functions.
  friend MotionModelInterface<LinearMotionModel<StateT>>;

  ///
  /// @brief      A crtp-called function that predicts the state forward.
  ///
  /// @param[in]  state  The current state vector
  /// @param[in]  dt     Time difference
  ///
  /// @return     New state after prediction.
  ///
  inline State crtp_predict(
    const State & state,
    const std::chrono::nanoseconds & dt) const
  {
    return State{crtp_jacobian(state, dt) * state.vector()};
  }

  ///
  /// @brief      A crtp-called function that computes a Jacobian.
  ///
  /// @note       The default implementation assumes that all variables have position, velocity and
  ///             acceleration entries. If a custom state that does not follow this convention is to
  ///             be used a specialization of this function must be added.
  ///
  /// @return     A matrix that represents the Jacobian.
  ///
  typename State::Matrix crtp_jacobian(const State &, const std::chrono::nanoseconds & dt) const;
};

}  // namespace motion_model
}  // namespace common
}  // namespace autoware

#endif  // MOTION_MODEL__LINEAR_MOTION_MODEL_HPP_
