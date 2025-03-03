// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef ACTUATION_AUTOWARE_CONFIG_HPP_
#define ACTUATION_AUTOWARE_CONFIG_HPP_

// Logging level
#define LOG_LEVEL LOG_LEVEL_DBG

// Thread stack size for nodes
#define THREAD_STACK_SIZE 10 * 1024

// DDS settings
#define DDS_DOMAIN_ID 2
#define ACTUATION_SERVICE_PORT 49152

// Packet analyzer
#define PACKET_ANALYZER_FIN "packet_analyzer_fin"
#define PACKET_ANALYZER_FIN_ACK "packet_analyzer_fin_ack"

#endif  // ACTUATION_AUTOWARE_CONFIG_HPP_
