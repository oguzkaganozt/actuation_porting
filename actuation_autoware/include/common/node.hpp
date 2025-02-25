#ifndef NODE_HPP
#define NODE_HPP

#include <zephyr/kernel.h>
#include "common/dds.hpp"

/**
 * @brief Node class implementation to represent ROS2 nodes within the Zephyr OS
 */
class Node {
public:
    Node(const char* name);
    ~Node();
    void conf_reader();
    

private:
    char name_[64];
    char topic_name_[64];
    char service_name_[64];
    char action_name_[64];
    char param_name_[64];
    char param_value_[64];
};

#endif
