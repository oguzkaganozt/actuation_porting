// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/motion_common/include/motion_common/trajectory_common.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, the Autoware Foundation
// Modifications: Copyright (c) 2022-2024, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTION_COMMON__TRAJECTORY_COMMON_HPP_
#define MOTION_COMMON__TRAJECTORY_COMMON_HPP_

#include <experimental/optional>
#include <limits>
#include <stdexcept>
#include <vector>

#include "TrajectoryPoint.h"
#include "common/types.hpp"
// newlib doesn't implement long double functions when sizeof(double) < sizeof(long double); needed
// by Eigen
long double __attribute__((weak)) fabsl(long double x) {
  return static_cast<long double>(fabsf(static_cast<double>(x)));
}
#include "Eigen/Core"
#include "geometry/common_2d.hpp"
#include "Pose.h"
#include "helper_functions/angle_utils.hpp"
#include "motion_common/motion_common.hpp"

namespace autoware
{
namespace motion
{
namespace motion_common
{
typedef autoware_auto_planning_msgs_msg_TrajectoryPoint Point;
typedef decltype (autoware_auto_planning_msgs_msg_Trajectory::points) Points;
using autoware::common::types::float64_t;
typedef Eigen::Matrix<float64_t, 3, 1> Vector3f;

/**
 * @brief throws an exception if the given list of points is empty
 * @param [in] points list of points to check
 */
MOTION_COMMON_PUBLIC void validateNonEmpty(const Points & points);

/**
 * @brief calculate the yaw deviation between two angles
 * @param [in] base_yaw base yaw angle [radians]
 * @param [in] target_yaw target yaw angle [radians]
 * @return normalized angle from the base to the target [radians]
 */
MOTION_COMMON_PUBLIC float64_t calcYawDeviation(
  const float64_t & base_yaw,
  const float64_t & target_yaw);

/**
 * @brief search first index with a velocity of zero in the given range of points
 * @param [in] points list of points to check
 * @param [in] src_idx starting search index
 * @param [in] dst_idx ending (excluded) search index
 * @param [in] epsilon optional value to use to determine zero velocities
 * @return index of the first zero velocity point found
 */
MOTION_COMMON_PUBLIC std::experimental::optional<size_t> searchZeroVelocityIndex(
  const Points & points, const size_t src_idx, const size_t dst_idx,
  const float64_t epsilon = 1e-3);

/**
 * @brief search first index with a velocity of zero in the given points
 * @param [in] points list of points to check
 */
MOTION_COMMON_PUBLIC std::experimental::optional<size_t> searchZeroVelocityIndex(
  const Points & points);

/**
 * @brief search the index of the point nearest to the given target with limits on the distance and yaw deviation
 * @param [in] points list of points to search
 * @param [in] pose target point
 * @param [in] max_dist optional maximum distance from the pose when searching for the nearest index
 * @param [in] max_yaw optional maximum deviation from the pose when searching for the nearest index
 * @return index of the point nearest to the target
 */
MOTION_COMMON_PUBLIC std::experimental::optional<size_t> findNearestIndex(
  const Points & points, const geometry_msgs_msg_Pose & pose,
  const float64_t max_dist = std::numeric_limits<float64_t>::max(),
  const float64_t max_yaw = std::numeric_limits<float64_t>::max());

/**
  * @brief calculate arc length along points
  * @param [in] points input points
  * @param [in] src_idx source index
  * @param [in] dst_idx destination index
  * @return arc length distance from source to destination along the input points
  */
MOTION_COMMON_PUBLIC float64_t calcSignedArcLength(
  const Points & points, const size_t src_idx,
  const size_t dst_idx);

}  // namespace motion_common
}  // namespace motion
}  // namespace autoware

#endif  // MOTION_COMMON__TRAJECTORY_COMMON_HPP_
