// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef DDS_HPP_
#define DDS_HPP_

#include <dds/ddsi/ddsi_config.h>

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
  cfg.participantIndex = DDSI_PARTICIPANT_INDEX_AUTO;
  cfg.maxAutoParticipantIndex = 60;
  cfg.allowMulticast = DDSI_AMC_SPDP;

  // Trace
  // cfg.tracefp = NULL;
  // cfg.tracemask = DDS_LC_FATAL | DDS_LC_ERROR;
  // cfg.tracefile = const_cast<char *>("stderr");
  // cfg.tracefp = NULL;

#if defined(CONFIG_DDS_NETWORK_INTERFACE)
  if (sizeof(CONFIG_DDS_NETWORK_INTERFACE) > 1) {
    cfg.network_interfaces = &cfg_iface;
  }
#endif
}

#endif  // DDS_HPP_
