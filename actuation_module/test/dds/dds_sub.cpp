#include "common/node/node.hpp"
#include "common/clock/clock.hpp"
#include "common/sequence/sequence.hpp"

// Msgs
#include "PoseStamped.h"
using PoseStampedMsg = geometry_msgs_msg_PoseStamped;

#if defined(NATIVE_SIM)
#define STACK_SIZE (4096)
static unsigned char node_stack[STACK_SIZE];
static unsigned char timer_stack[STACK_SIZE];
#else
static K_THREAD_STACK_DEFINE(node_stack, 4096);
static K_THREAD_STACK_DEFINE(timer_stack, 4096);
#define STACK_SIZE (K_THREAD_STACK_SIZEOF(node_stack))
#endif

/*
    This test is used to test the DDS communication between ROS2 and Zephyr
    It is used to validate the message conversion between ROS2 and Zephyr
    It is used to validate the sequence wrapper    
*/
static void handle_pose(PoseStampedMsg& msg) {
    printf("--------------------------------\n");
    printf("Timestamp: %ld\n", Clock::toDouble(msg.header.stamp));
    printf("Received pose: (%.1f, %.1f, %.1f)\n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    printf("--------------------------------\n");
}

int main(void) {
    printf("--------------------------------\n");
    printf("Starting DDS subscriber\n");
    printf("--------------------------------\n");
    printf("Waiting for Network interface to be ready\n");
    sleep(5);

    // Setting time using SNTP
    if (Clock::init_clock_via_sntp() < 0) {
        printf("Failed to set time using SNTP\n");
    }
    else {
        printf("Time set using SNTP\n");
    }
    
    // Create a subscriber for the test topic
    Node node("dds_test_sub", node_stack, STACK_SIZE, timer_stack, STACK_SIZE);
    auto subscriber = node.create_subscription<PoseStampedMsg>("test_pose", &geometry_msgs_msg_PoseStamped_desc, handle_pose);

    printf("--------------------------------\n");
    printf("DDS subscriber started\n");
    printf("--------------------------------\n");

    while(true) {
        sleep(1);
    }

    return 0;
}