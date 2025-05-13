// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_RESET "\033[0m"

#include <iostream>
#include "common/dds/dds_network.hpp"
#include "common/clock/clock.hpp"
#include "common/logger/logger.hpp"
using namespace common::logger;

#include "autoware/trajectory_follower_node/controller_node.hpp"

int main(void)
{   
    autoware::motion::control::trajectory_follower_node::Controller* controller;
    
    log_info("-----------------------------------------\n");
    log_info("ARM - Autoware: Actuation Safety Island\n");
    log_info("-----------------------------------------\n");
    sleep(5);

    // Setting time using SNTP
    log_info("Setting time using SNTP...\n");
    if (Clock::init_clock_via_sntp() < 0) {
        log_error("Failed to set time using SNTP\n");
        std::exit(1);
    }
    else {
        log_info("Time set using SNTP\n");
        log_info("-----------------------------------------\n");
        sleep(1);
    }

    // TODO: we are not configuring the network as we are using DHCP and other configurations will be done by cyclonedds
    // log_info("Configuring Network...\n");
    // if(configure_network()) {
    //     log_error("Failed to configure network\n");
    //     std::exit(1);
    // }
    // log_info("Network configured\n");

    log_info("Starting Controller Node...\n");
    try
    {
        controller = new autoware::motion::control::trajectory_follower_node::Controller();
        log_info("Controller Node Started\n");
        log_info("-----------------------------------------\n");
    }
    catch(const std::exception& e)
    {
        log_error("Failed to start Controller Node: %s\n", e.what());
        std::exit(1);
    }

    log_info("Actuation Safety Island is Live\n");
    log_info("-----------------------------------------\n");

    while (1) {
        sleep(1);
    }

    return 0;
}
