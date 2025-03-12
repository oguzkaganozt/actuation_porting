// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef BASE_DDS_IMPL_DDS_CONFIG_HPP_
#define BASE_DDS_IMPL_DDS_CONFIG_HPP_

#include <dds/ddsi/ddsi_config.h>
#include <dds/dds.h>

#define DDS_DOMAIN_ID 2
#define DDS_TRANSPORT_TYPE DDSI_TRANS_UDP // can be DDSI_TRANS_UDP or DDSI_TRANS_TCP
#define DDS_TCP_PORT 49152 // can be used when DDSI_TRANS_TCP

#if defined(CONFIG_DDS_NETWORK_INTERFACE)
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
#endif

/**
 * @brief Initialize a given DDS configuration structure.
 * @param[out] cfg Configuration structure that will be filled.
 */
inline static void init_config(struct ddsi_config & cfg)
{
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
  cfg.tracemask = DDS_LC_FATAL | DDS_LC_ERROR;
  cfg.tracefile = const_cast<char *>("stderr");

#if DDS_TRANSPORT_TYPE == DDSI_TRANS_TCP
  cfg.transport_selector = DDSI_TRANS_TCP;
  cfg.tcp_port = DDS_TCP_PORT;
#else
  cfg.transport_type = DDSI_TRANS_UDP;
#endif

#if defined(CONFIG_DDS_NETWORK_INTERFACE)
  if (sizeof(CONFIG_DDS_NETWORK_INTERFACE) > 1) {
    cfg.network_interfaces = &cfg_iface;
    printf("Network interface: %s\n", CONFIG_DDS_NETWORK_INTERFACE);
  }
#endif
}



#endif  // BASE_DDS_IMPL_DDS_CONFIG_HPP_
