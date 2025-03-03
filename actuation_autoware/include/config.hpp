// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef ACTUATION_AUTOWARE_CONFIG_HPP_
#define ACTUATION_AUTOWARE_CONFIG_HPP_

// Thread stack size for nodes
#define THREAD_STACK_SIZE 10 * 1024

// DDS settings
#define DDS_DOMAIN_ID 2
#define DDS_TRANSPORT_TYPE DDSI_TRANS_UDP // can be DDSI_TRANS_UDP or DDSI_TRANS_TCP
#define DDS_TCP_PORT 49152

#endif  // ACTUATION_AUTOWARE_CONFIG_HPP_
