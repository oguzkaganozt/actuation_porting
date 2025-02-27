// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef ACTUATION_AUTOWARE_HPP_
#define ACTUATION_AUTOWARE_HPP_

#include <zephyr/logging/log.h>

// Logging
#define LOG_LEVEL LOG_LEVEL_DBG

// Stack size
#define ACTUATION_STACK_SIZE 16 * 1024

// Domain ID used for Safety Island communication.
#define DDS_DOMAIN_ID 2
#define ACTUATION_SERVICE_PORT 49152

// Packet analyzer
#define PACKET_ANALYZER_FIN "packet_analyzer_fin"
#define PACKET_ANALYZER_FIN_ACK "packet_analyzer_fin_ack"

#endif  // ACTUATION_AUTOWARE_HPP_
