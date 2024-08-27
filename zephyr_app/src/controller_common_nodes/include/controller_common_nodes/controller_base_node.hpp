// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/controller_common_nodes/include/controller_common_nodes/controller_base_node.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, Christopher Ho
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
#ifndef CONTROLLER_COMMON_NODES__CONTROLLER_BASE_NODE_HPP_
#define CONTROLLER_COMMON_NODES__CONTROLLER_BASE_NODE_HPP_

#include <controller_common_nodes/visibility_control.hpp>
#include <Trajectory.h>
#include <VehicleKinematicState.h>
#include <controller_common/controller_base.hpp>

#include <dds/dds.h>

#include <exception>
#include <functional>
#include <memory>
#include <list>
#include <string>

#if defined CONTROL_CMDS_FWD_BSD_SOCKET
#include <pthread.h>
#endif

namespace motion
{
namespace control
{
namespace controller_common_nodes
{
using Command = autoware_auto_vehicle_msgs_msg_VehicleControlCommand;
using State = autoware_auto_vehicle_msgs_msg_VehicleKinematicState;
using Trajectory = autoware_auto_planning_msgs_msg_Trajectory;
using ControllerPtr = std::unique_ptr<controller_common::ControllerBase>;

class CONTROLLER_COMMON_NODES_PUBLIC ControllerBaseNode
{
public:
  /// Explicit constructor
  ControllerBaseNode(
    const std::string & name,
    const std::string & ns,
    const std::string & command_topic,
    const std::string & state_topic,
    const std::string & tf_topic,
    const std::string & trajectory_topic,
    const std::string & diagnostic_topic,
    const std::string & static_tf_topic = "static_tf");

  virtual ~ControllerBaseNode() noexcept;

protected:
  /// Child class should call this to set the controller
  void set_controller(ControllerPtr && controller) noexcept;
  /// Handles errors thrown by either check_new_trajectory(), handle_new_trajectory()
  /// or the std::domain_error from set_trajectory()
  virtual void on_bad_trajectory(std::exception_ptr eptr);
  /// Handles errors thrown by compute_command_impl(), or std::domain_error due to
  /// empty trajectories
  virtual void on_bad_compute(std::exception_ptr eptr);
  /// Expose publishing in case a child class wants to do something during error handling
  void publish(const Command & msg);
#if defined CONTROL_CMDS_FWD_BSD_SOCKET
  void init_analyzer_handler();
  void send_command(const Command & msg);
#endif


private:
  // Common initialization
  CONTROLLER_COMMON_NODES_LOCAL void init(
    const std::string & command_topic,
    const std::string & state_topic,
    const std::string & tf_topic,
    const std::string & static_tf_topic,
    const std::string & trajectory_topic,
    const std::string & diagnostic_topic);
  // Callbacks, note passing smart pointers by ref is fine if you're not using ownership
  // semantics:
  // stackoverflow.com/questions/3310737/
  // should-we-pass-a-shared-ptr-by-reference-or-by-value/8741626
  CONTROLLER_COMMON_NODES_LOCAL void on_trajectory(const Trajectory & msg);
  std::function<void(const Trajectory &)> m_trajectory_process {};
  CONTROLLER_COMMON_NODES_LOCAL void on_state(const State & msg);
  std::function<void(const State &)> m_state_process {};
  // Main computation, false if failure (due to missing tf?)
  CONTROLLER_COMMON_NODES_LOCAL bool try_compute(const State & state);
  // Try to compute control commands from old states in the context of new trajectories and tfs
  CONTROLLER_COMMON_NODES_LOCAL void retry_compute();
  CONTROLLER_COMMON_NODES_LOCAL void create_reader(
    const dds_topic_descriptor_t * desc,
    const char * name,
    const dds_qos_t * qos,
    dds_on_data_available_fn callback,
    void * arg);
  CONTROLLER_COMMON_NODES_LOCAL dds_entity_t create_writer(
    const dds_topic_descriptor_t * desc,
    const char * name,
    const dds_qos_t * qos);

  dds_entity_t m_participant {};
#if defined CONTROL_CMDS_FWD_BSD_SOCKET
  volatile int m_analyzer_cli_socket = 0;
  pthread_t m_analyzer_handle = {};
#elif defined CONTROL_CMDS_FWD_DDS
  dds_entity_t m_command_writer {};
#else
#error "Neither CONTROL_CMDS_FWD_BSD_SOCKET nor CONTROL_CMDS_FWD_DDS set"
#endif
  ControllerPtr m_controller{nullptr};
  std::list<State> m_uncomputed_states{};
  static constexpr size_t m_frame_size{5};
  char m_frame_id[m_frame_size]{};
};  // class ControllerBaseNode
}  // namespace controller_common_nodes
}  // namespace control
}  // namespace motion

#endif  // CONTROLLER_COMMON_NODES__CONTROLLER_BASE_NODE_HPP_
