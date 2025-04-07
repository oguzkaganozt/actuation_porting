// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_RESET "\033[0m"

#include <iostream>
#include "autoware/trajectory_follower_node/simple_trajectory_follower.hpp"
#include "autoware/trajectory_follower_node/controller_node.hpp"

void configure_network()
{
    //TODO: configure network
}

int main(void)
{   
    fprintf(stderr, COLOR_GREEN "-----------------------------------------\n" COLOR_RESET);
    fprintf(stderr, COLOR_GREEN "ARM - Autoware: Actuation Safety Island\n" COLOR_RESET);
    fprintf(stderr, COLOR_GREEN "-----------------------------------------\n" COLOR_RESET);
    sleep(2);

    fprintf(stderr, COLOR_GREEN "Starting Controller Node\n" COLOR_RESET);
    try
    {
        std::make_shared<autoware::motion::control::trajectory_follower_node::Controller>();
        sleep(2);
        fprintf(stderr, COLOR_GREEN "Controller Node Started\n" COLOR_RESET);
        fprintf(stderr, COLOR_GREEN "-----------------------------------------\n" COLOR_RESET);
    }
    catch(const std::exception& e)
    {
        std::cerr << COLOR_RED << "Failed to start Controller Node: " << e.what() << '\n' << COLOR_RESET;
        std::exit(1);
    }

    fprintf(stderr, COLOR_GREEN "Actuation Safety Island is Live\n" COLOR_RESET);
    fprintf(stderr, COLOR_GREEN "-----------------------------------------\n" COLOR_RESET);

    while (1) {
        sleep(1);
    }

    return 0;
}
