// Copyright 2015-2021 Autoware Foundation
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

#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include <string>

namespace autoware::vehicle_info_utils
{
VehicleInfoUtils::VehicleInfoUtils(
  double wheel_radius_m, double wheel_width_m, double wheel_base_m, double wheel_tread_m,
  double front_overhang_m, double rear_overhang_m, double left_overhang_m, double right_overhang_m,
  double vehicle_height_m, double max_steer_angle_rad
)
  : vehicle_info_{
      wheel_radius_m,
      wheel_width_m,
      wheel_base_m,
      wheel_tread_m,
      front_overhang_m,
      rear_overhang_m,
      left_overhang_m,
      right_overhang_m,
      vehicle_height_m,
      max_steer_angle_rad
    }
{
}

VehicleInfo VehicleInfoUtils::getVehicleInfo()
{
  return vehicle_info_;
}
}  // namespace autoware::vehicle_info_utils
