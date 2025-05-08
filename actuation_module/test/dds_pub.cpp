#include "common/node/node.hpp"
#include "common/clock/clock.hpp"
#include "common/sequence/sequence.hpp"
#include "common/logger/logger.hpp"
using namespace common::logger;

// Msgs
#include "PoseStamped.h"
using PoseStampedMsg = geometry_msgs_msg_PoseStamped;

#if defined(NATIVE_SIM)
static unsigned char node_stack[CONFIG_THREAD_STACK_SIZE];
static unsigned char timer_stack[CONFIG_THREAD_STACK_SIZE];
#else
static K_THREAD_STACK_DEFINE(node_stack, CONFIG_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(timer_stack, CONFIG_THREAD_STACK_SIZE);
#define STACK_SIZE (K_THREAD_STACK_SIZEOF(node_stack))
#endif

#define PUBLISH_PERIOD_MS (3000)

/*
    This test is used to test the DDS communication between ROS2 and Zephyr
    It is used to validate the message conversion between ROS2 and Zephyr
    It is used to validate the sequence wrapper    
*/
int main(void) {
    log_info("--------------------------------\n");
    log_info("Starting DDS publisher\n");
    log_info("--------------------------------\n");
    log_info("Waiting for Network interface to be ready\n");
    sleep(5);

    // Setting time using SNTP
    if (Clock::init_clock_via_sntp() < 0) {
        log_error("Failed to set time using SNTP\n");
    }
    else {
        log_info("Time set using SNTP\n");
    }

    // Create a publisher for the test topic
    Node node("dds_test_pub", node_stack, STACK_SIZE, timer_stack, STACK_SIZE);
    auto publisher = node.create_publisher<PoseStampedMsg>("test_pose", &geometry_msgs_msg_PoseStamped_desc);

    log_info("--------------------------------\n");
    log_info("DDS publisher started\n");
    log_info("--------------------------------\n");

    while(true) {
        PoseStampedMsg msg;
        msg.header.stamp = Clock::toRosTime(Clock::now());
        msg.header.frame_id = "map";
        msg.pose.position.x = 1.0;
        msg.pose.position.y = 2.0;
        msg.pose.position.z = 3.0;
        publisher->publish(msg);
        log_info("Published pose: (%.1f, %.1f, %.1f)\n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        sleep(PUBLISH_PERIOD_MS / 1000);
    }

    return 0;
}