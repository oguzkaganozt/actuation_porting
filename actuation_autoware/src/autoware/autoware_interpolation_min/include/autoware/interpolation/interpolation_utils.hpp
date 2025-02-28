// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__INTERPOLATION__INTERPOLATION_UTILS_HPP_
#define AUTOWARE__INTERPOLATION__INTERPOLATION_UTILS_HPP_

// Libs
#include "common/common.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>
#include <vector>

namespace autoware::interpolation
{
inline bool isIncreasing(const std::vector<double> & x)
{
  LOG_MODULE_DECLARE(autoware_interpolation);
  if (x.empty()) {
    LOG_ERR("Error: Points is empty.");
    return false;
  }

  for (size_t i = 0; i < x.size() - 1; ++i) {
    if (x.at(i) >= x.at(i + 1)) {
      return false;
    }
  }

  return true;
}

inline bool isNotDecreasing(const std::vector<double> & x)
{
  LOG_MODULE_DECLARE(autoware_interpolation);
  if (x.empty()) {
    LOG_ERR("Error: Points is empty.");
    return false;
  }

  for (size_t i = 0; i < x.size() - 1; ++i) {
    if (x.at(i) > x.at(i + 1)) {
      return false;
    }
  }

  return true;
}

inline std::vector<double> validateKeys(
  const std::vector<double> & base_keys, const std::vector<double> & query_keys)
{
  LOG_MODULE_DECLARE(autoware_interpolation);
  // when vectors are empty
  if (base_keys.empty() || query_keys.empty()) {
    LOG_ERR("Error: Points is empty.");
    return std::vector<double>();
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2) {
    LOG_ERR("Error: The size of points is less than 2. base_keys.size() = %d", base_keys.size());
    return std::vector<double>();
  }

  // when indices are not sorted
  if (!isIncreasing(base_keys) || !isNotDecreasing(query_keys)) {
    LOG_ERR("Error: Either base_keys or query_keys is not sorted.");
    return std::vector<double>();
  }

  // when query_keys is out of base_keys (This function does not allow exterior division.)
  constexpr double epsilon = 1e-3;
  if (
    query_keys.front() < base_keys.front() - epsilon ||
    base_keys.back() + epsilon < query_keys.back()) {
    LOG_ERR("Error: query_keys is out of base_keys");
    return std::vector<double>();
  }

  // NOTE: Due to calculation error of double, a query key may be slightly out of base keys.
  //       Therefore, query keys are cropped here.
  auto validated_query_keys = query_keys;
  validated_query_keys.front() = std::max(validated_query_keys.front(), base_keys.front());
  validated_query_keys.back() = std::min(validated_query_keys.back(), base_keys.back());

  return validated_query_keys;
}

template <class T>
void validateKeysAndValues(
  const std::vector<double> & base_keys, const std::vector<T> & base_values)
{
  LOG_MODULE_DECLARE(autoware_interpolation);
  // when vectors are empty
  if (base_keys.empty() || base_values.empty()) {
    LOG_ERR("Error: Points is empty.");
    return;
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2 || base_values.size() < 2) {
    LOG_ERR("Error: The size of points is less than 2. base_keys.size() = %d, base_values.size() = %d", base_keys.size(), base_values.size());
    return;
  }

  // when sizes of indices and values are not same
  if (base_keys.size() != base_values.size()) {
    LOG_ERR("Error: The size of base_keys and base_values are not the same.");
    return;
  }
}
}  // namespace autoware::interpolation

#endif  // AUTOWARE__INTERPOLATION__INTERPOLATION_UTILS_HPP_
