// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/controller_common_nodes/src/controller_common_nodes/controller_base_node.cpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, Christopher Ho
// Modifications: Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include "controller_common_nodes/controller_base_node.hpp"
#include "controller_common_nodes/endian_manip.hpp"
#include <motion_common/motion_common.hpp>
#include <time_utils/time_utils.hpp>
#include <zephyr_app.hpp>

#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#if defined CONTROL_CMDS_FWD_BSD_SOCKET
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>

static K_THREAD_STACK_DEFINE(analyzer_handler_area, ACTUATION_STACK_SIZE);
#endif

namespace motion
{
namespace control
{
namespace controller_common_nodes
{
////////////////////////////////////////////////////////////////////////////////
ControllerBaseNode::ControllerBaseNode(
  const std::string & name,
  const std::string & ns,
  const std::string & command_topic,
  const std::string & state_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic,
  const std::string & diagnostic_topic,
  const std::string & static_tf_topic)
{
  (void) name;
  (void) ns;
  init(command_topic, state_topic, tf_topic, static_tf_topic, trajectory_topic, diagnostic_topic);
  printf("Actuation Service initialized.\n");
}

ControllerBaseNode::~ControllerBaseNode()
{
  // Deleting the participant will delete all its children recursively as well.
  dds_delete(m_participant);
#if defined CONTROL_CMDS_FWD_BSD_SOCKET
  pthread_cancel(m_analyzer_handle);
#endif
}

static bool get_msg(dds_entity_t rd, void * sample)
{
  dds_sample_info_t info;
  dds_return_t rc = dds_take(rd, &sample, &info, 1, 1);
  if (rc < 0) {
    dds_log(DDS_LC_WARNING, __FILE__, __LINE__, DDS_FUNCTION, "can't take msg\n");
  }
  if (rc > 0 && info.valid_data) {
    return true;
  }
  return false;
}

template<typename T>
static void on_msg_dds(dds_entity_t rd, void * arg)
{
  if (!arg) {
    return;
  }
  auto process = *reinterpret_cast<std::function<void(const T &)> *>(arg);

  // Rely on CycloneDDS for the memory management of the message's sequence buffers.
  // A static message ensures the buffers allocated dynamically on the previous calls are not lost.
  static T msg;
  if (get_msg(rd, reinterpret_cast<void *>(&msg))) {
    process(msg);
  }
}

void ControllerBaseNode::create_reader(
  const dds_topic_descriptor_t * desc,
  const char * name, const dds_qos_t * qos,
  dds_on_data_available_fn callback, void * arg)
{
  dds_entity_t topic = dds_create_topic(m_participant, desc, name, NULL, NULL);
  if (topic < 0) {
    printf("dds_create_topic (%s): %s\n", name, dds_strretcode(-topic));
    std::exit(EXIT_FAILURE);
  }

  dds_listener_t * listener = dds_create_listener(arg);
  dds_lset_data_available(listener, callback);

  dds_entity_t reader = dds_create_reader(m_participant, topic, qos, listener);
  if (reader < 0) {
    printf("dds_create_reader (%s): %s\n", name, dds_strretcode(-reader));
    std::exit(EXIT_FAILURE);
  }

  dds_delete_listener(listener);
}

dds_entity_t ControllerBaseNode::create_writer(
  const dds_topic_descriptor_t * desc,
  const char * name, const dds_qos_t * qos)
{
  dds_entity_t topic = dds_create_topic(m_participant, desc, name, NULL, NULL);
  if (topic < 0) {
    printf("dds_create_topic (%s): %s\n", name, dds_strretcode(-topic));
    std::exit(EXIT_FAILURE);
  }

  dds_entity_t writer = dds_create_writer(m_participant, topic, qos, NULL);
  if (writer < 0) {
    printf("dds_create_writer (%s): %s\n", name, dds_strretcode(-writer));
    std::exit(EXIT_FAILURE);
  }

  return writer;
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::init(
  const std::string & command_topic,
  const std::string & state_topic,
  const std::string & tf_topic,
  const std::string & static_tf_topic,
  const std::string & trajectory_topic,
  const std::string & diagnostic_topic)
{
  (void) diagnostic_topic;
  // Common error checking
  if (command_topic.empty()) {
    throw std::domain_error{"Command topic not set"};
  }
  if (state_topic.empty()) {
    throw std::domain_error{"State topic not set"};
  }
  if (tf_topic.empty()) {
    throw std::domain_error{"TF topic not set"};
  }
  if (static_tf_topic.empty()) {
    throw std::domain_error{"Static TF topic not set"};
  }
  if (trajectory_topic.empty()) {
    throw std::domain_error{"Trajectory topic not set"};
  }

  struct ddsi_config dds_cfg;
  init_config(dds_cfg);

  dds_entity_t domain = dds_create_domain_with_rawconfig(DDS_DOMAIN_ACTUATION, &dds_cfg);
  // The domain could have been set prior to this point, don't fail in this case.
  if (domain < 0 && domain != DDS_RETCODE_PRECONDITION_NOT_MET) {
    printf("dds_create_domain_with_rawconfig: %s\n", dds_strretcode(-domain));
    std::exit(EXIT_FAILURE);
  }

  m_participant = dds_create_participant(DDS_DOMAIN_ACTUATION, NULL, NULL);
  if (m_participant < 0) {
    printf("dds_create_participant: %s\n", dds_strretcode(-m_participant));
    std::exit(EXIT_FAILURE);
  }
  dds_qos_t * qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));

  m_state_process = [&](const State & msg) {on_state(msg);};
  create_reader(
    &autoware_auto_vehicle_msgs_msg_VehicleKinematicState_desc,
    state_topic.c_str(),
    qos,
    on_msg_dds<State>,
    reinterpret_cast<void *>(&m_state_process)
  );

  m_trajectory_process = [&](const Trajectory & msg) {on_trajectory(msg);};
  create_reader(
    &autoware_auto_planning_msgs_msg_Trajectory_desc,
    trajectory_topic.c_str(),
    qos,
    on_msg_dds<Trajectory>,
    reinterpret_cast<void *>(&m_trajectory_process)
  );

#if defined CONTROL_CMDS_FWD_BSD_SOCKET
  init_analyzer_handler();
#elif defined CONTROL_CMDS_FWD_DDS
  m_command_writer = create_writer(
    &autoware_auto_vehicle_msgs_msg_VehicleControlCommand_desc,
    command_topic.c_str(),
    qos
  );
#else
#error "Neither CONTROL_CMDS_FWD_BSD_SOCKET nor CONTROL_CMDS_FWD_DDS set"
#endif

  dds_delete_qos(qos);
}
////////////////////////////////////////////////////////////////////////////////
#if defined CONTROL_CMDS_FWD_BSD_SOCKET
static bool read_all(int cli_socket, uint8_t *buf, size_t buf_len)
{
  bool ret = true;

  // Return false if the receiver has closed the socket
  while (buf_len > 0)
  {
    ssize_t bytes_read = read(cli_socket, buf, buf_len);
    if (bytes_read < 1) {
      printf("ERROR: read_all failed, bytes_read: %zd, errno: %d\n",
             bytes_read, errno);
      ret = false;
      break;
    }
    buf += bytes_read;
    buf_len -= bytes_read;
  }

  return ret;
}
////////////////////////////////////////////////////////////////////////////////
static bool send_all(int cli_socket, uint8_t * buf, size_t buf_len)
{
  bool ret = true;

  // Return false if the receiver has closed the socket
  while (buf_len > 0)
  {
    ssize_t bytes_sent = send(cli_socket, buf, buf_len, 0);

    if (bytes_sent < 1) {
      printf("ERROR: send_all failed, bytes_sent: %zd, errno: %d",
             bytes_sent, errno);
      ret = false;
      break;
    }
    buf += bytes_sent;
    buf_len -= bytes_sent;
  }

  return ret;
}
////////////////////////////////////////////////////////////////////////////////
static void * get_analyzer_handle(void * arg)
{
  volatile int *analyzer_cli_socket =  static_cast<volatile int *>(arg);
  int rc = 0;
  int srv_socket;
  int srv_port = ACTUATION_SERVICE_PORT;
  std::stringstream ss;

  srv_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (srv_socket < 0 ) {
    ss << "Actuation Service, socket create failed with error: " << errno;
    throw std::domain_error{ss.str()};
  }

  struct sockaddr_in srv_info;
  memset(&srv_info, 0, sizeof(srv_info));
  srv_info.sin_family = AF_INET;
  srv_info.sin_port = htons(srv_port);
  srv_info.sin_addr.s_addr = htonl(INADDR_ANY);

  rc = bind(srv_socket, (const struct sockaddr *)&srv_info,
            sizeof(srv_info));
  if (rc != 0 ) {
    ss << "Actuation Service, socket bind failed with error: " << errno;
    throw std::domain_error{ss.str()};
  }

  // The Actuation Service can serve at max one Packet Analyzer
  rc = listen(srv_socket, 1);
  if (rc != 0) {
    ss << "Actuation Service, socket listen failed with error: " << errno;
    throw std::domain_error{ss.str()};
  }

  // Performing accept in a while true loop. This is to facilitate accepting
  // of connection from the Packet Analyzer multiple times; but keeping
  // max connection at any point of time to 1
  struct sockaddr_in cli_info;
  socklen_t cli_info_len = sizeof(cli_info);
  while (true) {
    if (*analyzer_cli_socket) {
      k_msleep(1000);
      continue;
    }

    printf("Thread get_analyzer_handle performing a blocking accept\n");
    memset(&cli_info, 0, cli_info_len);
    *analyzer_cli_socket = accept(srv_socket, (struct sockaddr*)&cli_info,
                                (socklen_t*)&cli_info_len);
    printf("Accepted tcp connection from the Packet Analyzer: <%d>\n",
           *analyzer_cli_socket);

    if (*analyzer_cli_socket < 0) {
      ss << "Actuation Service, socket accept failed with error: " << errno;
      throw std::domain_error{ss.str()};
    }

    // Check if the Packet Analyzer has sent a FIN
    std::array<uint8_t, sizeof(Command)> buf;
    // read_all can fail in the following scenarios:
    // - Packet Analyzer has stopped working
    // - Comm between Packet Analyzer and Actuation Server is no longer working
    // - Actuation Service closed client connection during a send_all failure
    if (read_all(*analyzer_cli_socket, buf.data(), sizeof(Command))) {
      if (0 == memcmp(buf.data(), PACKET_ANALYZER_FIN,
                      sizeof(PACKET_ANALYZER_FIN))) {
        // Close the client connection after sending a FIN_ACK
        memset(buf.data(), 0, sizeof(Command));
        memcpy(buf.data(), PACKET_ANALYZER_FIN_ACK,
               sizeof(PACKET_ANALYZER_FIN_ACK));

        // Not checking the return value as the client has initiated a FIN
        send_all(*analyzer_cli_socket, buf.data(), sizeof(Command));
        close(*analyzer_cli_socket);
        *analyzer_cli_socket = 0;
      }
      else {
        printf("get_analyzer_handle, ignoring rx packet from client\n");
      }
    }
    else {
      printf("get_analyzer_handle, closing client socket as read failed\n");
      close(*analyzer_cli_socket);
      *analyzer_cli_socket = 0;
    }
  }

  return NULL;
}
////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::init_analyzer_handler()
{
  std::stringstream ss;
  pthread_attr_t analyzer_handler_attr {};
  size_t analyzer_handler_stacksz = K_THREAD_STACK_SIZEOF(analyzer_handler_area);
  int rc = pthread_attr_init(&analyzer_handler_attr);
  if (rc != 0 ) {
    ss << "init_analyzer_handler: pthread_attr_init failed: " << errno;
    throw std::domain_error{ss.str()};
  }

  rc = pthread_attr_setstack(&analyzer_handler_attr, &analyzer_handler_area,
                             analyzer_handler_stacksz);
  if (rc != 0 ) {
    ss << "init_analyzer_handler: pthread_attr_setstack failed: " << errno;
    throw std::domain_error{ss.str()};
  }

  rc = pthread_create(&m_analyzer_handle, &analyzer_handler_attr,
                      &get_analyzer_handle, (void *) &m_analyzer_cli_socket);
  if (rc != 0 ) {
    ss << "init_analyzer_handler: pthread_create failed: " << errno;
    throw std::domain_error{ss.str()};
  }


  // Calling yield to give get_analyzer_handle a chance to setup socket and
  // perform a blocking accept
  sched_yield();
}
////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::send_command(const Command & msg)
{

  if (m_analyzer_cli_socket) {

    // Use little endian while sending control commands to host
    std::array<uint8_t, sizeof(Command)> msg_buf;
    uint8_t * cursor = msg_buf.data();

    // Serialize control commands to little endian format
    cursor = ser_uint32(msg.stamp.sec, cursor);
    cursor = ser_uint32(msg.stamp.nanosec, cursor);
    cursor = ser_float(msg.long_accel_mps2, cursor);
    cursor = ser_float(msg.velocity_mps, cursor);
    cursor = ser_float(msg.front_wheel_angle_rad, cursor);
    cursor = ser_float(msg.rear_wheel_angle_rad, cursor);

    if (!send_all(m_analyzer_cli_socket, msg_buf.data(), sizeof(Command))) {
      close(m_analyzer_cli_socket);
      m_analyzer_cli_socket = 0;
    }
  }
}
#endif
////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::retry_compute()
{
  // Really inelegant mechanism, but that's callbacks, need to maintain throughput
  while (!m_uncomputed_states.empty()) {
    const auto & state = m_uncomputed_states.front();
    if (!try_compute(state)) {
      break;
    }
    m_uncomputed_states.pop_front();
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_trajectory(const Trajectory & msg)
{
  try {
    m_controller->set_trajectory(msg);
    // Only retry computation if new trajectory was successfully set
    retry_compute();
  } catch (...) {
    on_bad_trajectory(std::current_exception());
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_state(const State & msg)
{
  if (!try_compute(msg)) {
    // Only keep one element to avoid memory overflow when not receiving trajectories.
    // No point in having a list except for minimizing changes to Autoware code.
    m_uncomputed_states.clear();
    m_uncomputed_states.push_back(msg);
    // Copy frame_id content as IDLC uses pointers for strings.
    std::strncpy(m_frame_id, msg.header.frame_id, m_frame_size - 1);
    m_uncomputed_states.front().header.frame_id = m_frame_id;
  }
}
////////////////////////////////////////////////////////////////////////////////
bool ControllerBaseNode::try_compute(const State & state)
{
  if (!state.header.frame_id || state.header.frame_id[0] == '\0') {
    dds_log(
      DDS_LC_WARNING, __FILE__, __LINE__, DDS_FUNCTION,
      "try_compute: empty state frame, ignoring\n");
    return true;
  }
  if (!m_controller->get_reference_trajectory().header.frame_id ||
    m_controller->get_reference_trajectory().header.frame_id[0] == '\0')
  {
    // TODO(Takamasa Horibe): Enable with RCLCPP_WARN_THROTTLE after Foxy
    // RCLCPP_WARN(
    //   get_logger(),
    //   "try_compute: empty trajectory frame, possibly uninitialized, deferring");
    return false;
  }

  // Compute result
  try {
    const auto cmd{m_controller->compute_command(state)};
    publish(cmd);
  } catch (...) {
    on_bad_compute(std::current_exception());
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::publish(const Command & msg)
{
  auto ts = static_cast<unsigned long long>(msg.stamp.sec) * 1000000000 + msg.stamp.nanosec;
  printf("%llu: %7.4f (m/s^2) | %7.4f (rad)\n", ts, msg.long_accel_mps2, msg.front_wheel_angle_rad);
#if defined CONTROL_CMDS_FWD_BSD_SOCKET
  send_command(msg);
#elif defined CONTROL_CMDS_FWD_DDS
  dds_write(m_command_writer, &msg);
#else
#error "Neither CONTROL_CMDS_FWD_BSD_SOCKET nor CONTROL_CMDS_FWD_DDS set"
#endif
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::set_controller(ControllerPtr && controller) noexcept
{
  m_controller = std::forward<ControllerPtr &&>(controller);
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_bad_trajectory(std::exception_ptr eptr)  // NOLINT
{
  // Pass-by-reference is generally inappropriate for pointer types
  try {
    if (eptr) {
      std::rethrow_exception(eptr);
    } else {
      dds_log(DDS_LC_WARNING, __FILE__, __LINE__, DDS_FUNCTION, "on_bad_trajectory: nullptr\n");
    }
  } catch (const std::exception & e) {
    dds_log(DDS_LC_WARNING, __FILE__, __LINE__, DDS_FUNCTION, "%s\n", e.what());
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_bad_compute(std::exception_ptr eptr)  // NOLINT
{
  // Pass-by-reference is generally inappropriate for pointer types
  try {
    if (eptr) {
      std::rethrow_exception(eptr);
    } else {
      dds_log(DDS_LC_WARNING, __FILE__, __LINE__, DDS_FUNCTION, "on_bad_compute: nullptr\n");
    }
  } catch (const std::exception & e) {
    dds_log(DDS_LC_WARNING, __FILE__, __LINE__, DDS_FUNCTION, "%s\n", e.what());
  }
}

}  // namespace controller_common_nodes
}  // namespace control
}  // namespace motion
