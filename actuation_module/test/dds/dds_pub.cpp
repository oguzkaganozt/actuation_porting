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
int main(void) {
    printf("--------------------------------\n");
    printf("Starting DDS publisher\n");
    printf("--------------------------------\n");
    printf("Waiting for Network interface to be ready\n");
    sleep(5);

    // Create a publisher for the test topic
    Node node("dds_test_pub", node_stack, STACK_SIZE, timer_stack, STACK_SIZE);
    auto publisher = node.create_publisher<PoseStampedMsg>("test_pose", &geometry_msgs_msg_PoseStamped_desc);

    printf("--------------------------------\n");
    printf("DDS publisher started\n");
    printf("--------------------------------\n");

    while(true) {
        PoseStampedMsg msg;
        msg.header.stamp = Clock::toRosTime(Clock::now());
        msg.header.frame_id = "map";
        msg.pose.position.x = 1.0;
        msg.pose.position.y = 2.0;
        msg.pose.position.z = 3.0;
        publisher->publish(msg);
        printf("Published pose: (%.1f, %.1f, %.1f)\n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        sleep(3);
    }

    return 0;
}