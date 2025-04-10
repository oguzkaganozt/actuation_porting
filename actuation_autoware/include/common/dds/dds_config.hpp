// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef COMMON__DDS_CONFIG_HPP_
#define COMMON__DDS_CONFIG_HPP_

#include <dds/ddsi/ddsi_config.h>
#include <dds/dds.h>
#include "config.hpp"

static struct ddsi_config_network_interface_listelem cfg_iface
{
  nullptr,
  {
    0,
    const_cast<char *>(CONFIG_DDS_NETWORK_INTERFACE),
    nullptr,
    0,
    1,
    DDSI_BOOLDEF_DEFAULT,
    {1, 0}
  }
};

/**
 * @brief Initialize a given DDS configuration structure.
 * @param[out] cfg Configuration structure that will be filled.
 */
inline static void init_config(struct ddsi_config & cfg)
{
  fprintf(stderr, "Initializing DDS configuration\n");

  if (sizeof(CONFIG_DDS_NETWORK_INTERFACE) <= 1) {
    fprintf(stderr, "DDS network interface not set, please set CONFIG_DDS_NETWORK_INTERFACE\n");
    std::exit(1);
  }
  else {
    fprintf(stderr, "Network interface: %s\n", CONFIG_DDS_NETWORK_INTERFACE);
  }

  ddsi_config_init_default(&cfg);

  // Buffers
  cfg.rbuf_size = 16 * 1024;
  cfg.rmsg_chunk_size = 2 * 1204;
  cfg.max_msg_size = 1456;

  // Discovery
#ifndef NATIVE_SIM
    cfg.participantIndex = DDSI_PARTICIPANT_INDEX_AUTO;
    cfg.maxAutoParticipantIndex = 60;
    cfg.allowMulticast = DDSI_AMC_SPDP;
#endif

  // Trace
  cfg.tracefp = NULL;
  cfg.tracemask = DDS_LC_FATAL | DDS_LC_ERROR | DDS_LC_WARNING | DDS_LC_CONFIG | DDS_LC_INFO; // DDS_LC_FATAL | DDS_LC_ERROR; // DDS_LC_ALL
  cfg.tracefile = const_cast<char *>("stderr");

  // Network interface
  cfg.network_interfaces = &cfg_iface;

  // if (DDS_TRANSPORT_TYPE == DDSI_TRANS_TCP) {
  //   cfg.transport_selector = DDSI_TRANS_TCP;
  //   cfg.tcp_port = DDS_TCP_PORT;
  //   fprintf(stderr, "Transport type: TCP, port: %d\n", DDS_TCP_PORT);
  // } else {
  //   cfg.transport_selector = DDSI_TRANS_UDP;
  //   fprintf(stderr, "Transport type: UDP\n");
  // }
}

#endif  // COMMON__DDS_CONFIG_HPP_
