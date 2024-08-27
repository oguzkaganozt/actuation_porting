// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/pure_pursuit_nodes/include/pure_pursuit_nodes/pure_pursuit_node.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_
#define PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_

#include <controller_common_nodes/controller_base_node.hpp>
#include <pure_pursuit/pure_pursuit.hpp>

#include <string>

#include "pure_pursuit_nodes/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Boilerplate Apex.OS nodes around pure_pursuit
namespace pure_pursuit_nodes
{
/// \brief Boilerplate node that subscribes to the current pose and
/// publishes a vehicle control command
class PURE_PURSUIT_NODES_PUBLIC PurePursuitNode
  : public ::motion::control::controller_common_nodes::ControllerBaseNode
{
public:
  /// \brief Parameter constructor
  /// \param[in] node_name Name of the node, controls which parameter set from the file is matched
  /// \param[in] node_namespace Name of the node's namespace, controls which parameters are used
  PurePursuitNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  /// \brief Explicit constructor
  /// \param[in] node_name Name of the node
  /// \param[in] cfg Configuration object for PurePursuit
  /// \param[in] node_namespace Namespace of this node
  PurePursuitNode(
    const std::string & node_name,
    const pure_pursuit::Config & cfg,
    const std::string & node_namespace = "");
};
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_
