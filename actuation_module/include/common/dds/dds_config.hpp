// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef COMMON__DDS_CONFIG_HPP_
#define COMMON__DDS_CONFIG_HPP_

#include <dds/ddsi/ddsi_config.h>
#include <dds/dds.h>
#include "config.hpp"

#if defined(CONFIG_NET_CONFIG_PEER_IPV4_ADDR)
static struct ddsi_config_peer_listelem cfg_peer
{
  nullptr,
  const_cast<char *>(CONFIG_NET_CONFIG_PEER_IPV4_ADDR)
};
#endif

static struct ddsi_config_network_interface_listelem cfg_iface
{
  nullptr,
  {
    0,
    const_cast<char *>(CONFIG_DDS_NETWORK_INTERFACE),
    nullptr,
    1,  // prefer_multicast, TODO: Enabled multicast, check if this is required
    1,
    DDSI_BOOLDEF_DEFAULT, // multicast
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

  // TODO: Check if this is required
  // cfg.retransmit_merging = DDSI_REXMIT_MERGE_ALWAYS;
  // cfg.multiple_recv_threads = DDSI_BOOLDEF_FALSE;

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
  cfg.tracefile = const_cast<char *>("stderr");
  #if CONFIG_DDS_LOG_LEVEL == 2
    cfg.tracemask = DDS_LC_ALL;
  #elif CONFIG_DDS_LOG_LEVEL == 1
    cfg.tracemask = DDS_LC_FATAL | DDS_LC_ERROR | DDS_LC_WARNING | DDS_LC_CONFIG ;
  #else
    cfg.tracemask = 0;
  #endif

  // Network interface
  cfg.network_interfaces = &cfg_iface;

#if defined(CONFIG_NET_CONFIG_PEER_IPV4_ADDR)
  if (sizeof(CONFIG_NET_CONFIG_PEER_IPV4_ADDR) > 1) {
    cfg.peers = &cfg_peer;
    fprintf(stderr, "Adding peer: %s\n", CONFIG_NET_CONFIG_PEER_IPV4_ADDR);
  }
#endif

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
