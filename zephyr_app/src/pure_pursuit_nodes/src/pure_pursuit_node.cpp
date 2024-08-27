// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/pure_pursuit_nodes/src/pure_pursuit_node.cpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, the Autoware Foundation
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>
#include <motion_common/motion_common.hpp>

#include <memory>
#include <string>

#include "pure_pursuit_nodes/pure_pursuit_node.hpp"

using autoware::common::types::float64_t;
using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the pure pursuit node package
namespace pure_pursuit_nodes
{
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const std::string & node_namespace)
: ControllerBaseNode{node_name, node_namespace, "ctrl_cmd", "current_pose",
    "tf", "trajectory", "ctrl_diag"}
{
  pure_pursuit::Config cfg{
    6.0F,  // controller.minimum_lookahead_distance
    100.0F,  // controller.maximum_lookahead_distance
    2.0F,  // controller.speed_to_lookahead_ratio
    true,  // controller.is_interpolate_lookahead_point
    false,  // controller.is_delay_compensation
    0.1F,  // controller.emergency_stop_distance
    0.3F,  // controller.speed_thres_traveling_direction
    2.7F  // controller.dist_front_rear_wheels
  };
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
////////////////////////////////////////////////////////////////////////////////
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const pure_pursuit::Config & cfg,
  const std::string & node_namespace)
: ControllerBaseNode{node_name, node_namespace, "ctrl_cmd", "current_pose",
    "tf", "trajectory", "ctrl_diag"}
{
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware
