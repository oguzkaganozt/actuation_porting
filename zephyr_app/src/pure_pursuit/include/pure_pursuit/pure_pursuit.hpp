// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/pure_pursuit/include/pure_pursuit/pure_pursuit.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, the Autoware Foundation
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef PURE_PURSUIT__PURE_PURSUIT_HPP_
#define PURE_PURSUIT__PURE_PURSUIT_HPP_

#include <Trajectory.h>
#include <TrajectoryPoint.h>
#include <VehicleKinematicState.h>
#include <VehicleControlCommand.h>
#include <controller_common/controller_base.hpp>
#include <utility>
#include "pure_pursuit/config.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the pure pursuit package
namespace pure_pursuit
{
using Trajectory = autoware_auto_planning_msgs_msg_Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs_msg_TrajectoryPoint;
using TrajectoryPointStamped = autoware_auto_vehicle_msgs_msg_VehicleKinematicState;
using VehicleControlCommand = autoware_auto_vehicle_msgs_msg_VehicleControlCommand;

/// \brief Given a trajectory and the current state, compute the control command
class PURE_PURSUIT_PUBLIC PurePursuit
  : public ::motion::control::controller_common::ControllerBase
{
public:
  /// \brief Default constructor
  /// \param[in] cfg Pure pursuit configuration parameters
  explicit PurePursuit(const Config & cfg);

protected:
  /// \brief Compute the vehicle command based on the current pose and the given trajectory.
  ///        If the trajectory's size is 0 or the current pose passed through the trajectory,
  ///        the emergency stop acceleration will be computed
  /// \param[in] state The current position and velocity information
  /// \return the command for the vehicle control
  VehicleControlCommand compute_command_impl(const TrajectoryPointStamped & state) override;

private:
  /// \brief Compute error of the current vehicle state by comparing the nearest neighbor
  ///        trajectory point
  /// \param[in] current_point The current position and velocity information
  PURE_PURSUIT_LOCAL void compute_errors(const TrajectoryPoint & current_point);
  /// \brief Compute the lookahead distance using the current velocity and the conversion ratio
  ///        from the velocity to distance
  /// \param[in] current_velocity The current vehicle velocity
  PURE_PURSUIT_LOCAL void compute_lookahead_distance(const float32_t current_velocity);
  /// \brief Whether the point is in the traveling direction.
  ///        If the value of the target velocity computed the previous update is positive,
  ///        the current target should be in the forward direction
  /// \param[in] current_point The current position and velocity information
  /// \param[in] point The candidate point in the trajectory
  /// \return True if the point is in the traveling direction
  PURE_PURSUIT_LOCAL bool8_t in_traveling_direction(
    const TrajectoryPoint & current_point,
    const TrajectoryPoint & point) const;
  /// \brief Get the interpolated target point by computing the intersection between
  ///        line from the candidate target point to its previous point, and the circle.
  ///        The center of a circle is the current pose and the lookahead distance is a radius
  /// \param[in] current_point The current position and velocity information
  /// \param[in] target_point The candidate target point
  /// \param[in] idx The candidate target index in the trajectory
  PURE_PURSUIT_LOCAL void compute_interpolate_target_point(
    const TrajectoryPoint & current_point,
    const TrajectoryPoint & target_point,
    const uint32_t idx);
  /// \brief Compute the target point using the current pose and the trajectory
  /// \param[in] current_point The current position and velocity information
  /// \return True if the controller get the current target point
  PURE_PURSUIT_LOCAL bool8_t compute_target_point(const TrajectoryPoint & current_point);
  /// \brief Compute the 2D distance between given two points
  /// \param[in] point1 The point with x and y position information
  /// \param[in] point2 The point with x and y position information
  /// \return the distance between given two points
  PURE_PURSUIT_LOCAL static float32_t compute_points_distance_squared(
    const TrajectoryPoint & point1,
    const TrajectoryPoint & point2);
  /// \brief Compute the relative y coordinate distance between two points
  /// \param[in] current The point with x and y position, and 2D heading information
  /// \param[in] target The point with x and y position, and 2D heading information
  /// \return the pair of relative x and y distances
  PURE_PURSUIT_LOCAL std::pair<float32_t, float32_t> compute_relative_xy_offset(
    const TrajectoryPoint & current,
    const TrajectoryPoint & target) const;
  /// \brief Compute the steering angle (radian) using the current pose and the target point
  /// \param[in] current_point The current position and velocity information
  /// \return the computed steering angle (radian)
  PURE_PURSUIT_LOCAL float32_t compute_steering_rad(const TrajectoryPoint & current_point);
  /// \brief Compute the acceleration command (mps) using the current pose and the target point
  /// \param[in] current_point The current position and velocity information
  /// \param[in] is_emergency The boolean whether the current state is emergency mode or not
  /// \return the computed acceleration
  PURE_PURSUIT_LOCAL float32_t compute_command_accel_mps(
    const TrajectoryPoint & current_point,
    const bool8_t is_emergency) const;

  float32_t m_lookahead_distance;
  TrajectoryPoint m_target_point;
  VehicleControlCommand m_command;
  Config m_config;

  uint32_t m_iterations;
};  // class PurePursuit
}  // namespace pure_pursuit
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // PURE_PURSUIT__PURE_PURSUIT_HPP_
