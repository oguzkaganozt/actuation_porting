// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_common/include/helper_functions/mahalanobis_distance.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2021, Apex.AI, Inc.
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef HELPER_FUNCTIONS__MAHALANOBIS_DISTANCE_HPP_
#define HELPER_FUNCTIONS__MAHALANOBIS_DISTANCE_HPP_

#include <Eigen/Cholesky>

namespace autoware
{
namespace common
{
namespace helper_functions
{
/// \brief Calculate square of mahalanobis distance
/// \tparam T Type of elements in the matrix
/// \tparam kNumOfStates Number of states
/// \param sample Single column matrix containing sample whose distance needs to be computed
/// \param mean Single column matrix containing mean of samples received so far
/// \param covariance_factor Covariance matrix
/// \return Square of mahalanobis distance
template<typename T, std::int32_t kNumOfStates>
types::float32_t calculate_squared_mahalanobis_distance(
  const Eigen::Matrix<T, kNumOfStates, 1> & sample,
  const Eigen::Matrix<T, kNumOfStates, 1> & mean,
  const Eigen::Matrix<T, kNumOfStates, kNumOfStates> & covariance_factor)
{
  using Vector = Eigen::Matrix<T, kNumOfStates, 1>;
  // This is equivalent to the squared Mahalanobis distance of the form: diff.T * C.inv() * diff
  // Instead of the covariance matrix C we have its lower-triangular factor L, such that C = L * L.T
  // squared_mahalanobis_distance = diff.T * C.inv() * diff
  // = diff.T * (L * L.T).inv() * diff
  // = diff.T * L.T.inv() * L.inv() * diff
  // = (L.inv() * diff).T * (L.inv() * diff)
  // this allows us to efficiently find the squared Mahalanobis distance using (L.inv() * diff),
  // which can be found as a solution to: L * x = diff.
  const Vector diff = sample - mean;
  const Vector x = covariance_factor.ldlt().solve(diff);
  return x.transpose() * x;
}

/// \brief Calculate mahalanobis distance
/// \tparam T Type of elements in the matrix
/// \tparam kNumOfStates Number of states
/// \param sample Single column matrix containing sample whose distance needs to be computed
/// \param mean Single column matrix containing mean of samples received so far
/// \param covariance_factor Covariance matrix
/// \return Mahalanobis distance
template<typename T, std::int32_t kNumOfStates>
types::float32_t calculate_mahalanobis_distance(
  const Eigen::Matrix<T, kNumOfStates, 1> & sample,
  const Eigen::Matrix<T, kNumOfStates, 1> & mean,
  const Eigen::Matrix<T, kNumOfStates, kNumOfStates> & covariance_factor
)
{
  return sqrtf(calculate_squared_mahalanobis_distance(sample, mean, covariance_factor));
}
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__MAHALANOBIS_DISTANCE_HPP_
