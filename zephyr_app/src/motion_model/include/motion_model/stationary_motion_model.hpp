// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/motion_model/include/motion_model/stationary_motion_model.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Developed by Apex.AI, Inc.

#ifndef MOTION_MODEL__STATIONARY_MOTION_MODEL_HPP_
#define MOTION_MODEL__STATIONARY_MOTION_MODEL_HPP_

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
/// @brief      This class describes a stationary motion model, i.e., the model that does not change
///             the state.
///
/// @tparam     StateT  State with which the motion model works.
///
template<typename StateT>
class MOTION_MODEL_PUBLIC StationaryMotionModel
  : public MotionModelInterface<StationaryMotionModel<StateT>>
{
public:
  using State = StateT;

protected:
  // Allow the CRTP interface to call private functions.
  friend MotionModelInterface<StationaryMotionModel<StateT>>;

  ///
  /// @brief      A crtp-called function that predicts the state forward.
  ///
  /// @param[in]  state  The current state vector
  ///
  /// @return     A const reference to the unchanged input state.
  ///
  inline const State & crtp_predict(
    const State & state,
    const std::chrono::nanoseconds &) const
  {
    return state;
  }

  ///
  /// @brief      A crtp-called function that computes a Jacobian for the stationary motion model.
  ///
  /// @return     An identity matrix.
  ///
  typename State::Matrix crtp_jacobian(const State &, const std::chrono::nanoseconds &) const
  {
    return Eigen::Matrix<typename State::Scalar, State::size(), State::size()>::Identity();
  }
};

}  // namespace motion_model
}  // namespace common
}  // namespace autoware

#endif  // MOTION_MODEL__STATIONARY_MOTION_MODEL_HPP_
