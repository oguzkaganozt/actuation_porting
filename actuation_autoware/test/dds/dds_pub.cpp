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

static void timer_callback(void* arg) {
    printf("Publishing message\n");
    auto publisher = static_cast<Publisher<PoseStampedMsg>*>(arg);
    PoseStampedMsg msg;
    msg.header.stamp = Clock::toRosTime(Clock::now());
    msg.header.frame_id = "map";
    msg.pose.position.x = 1.0;
    msg.pose.position.y = 2.0;
    msg.pose.position.z = 3.0;
    publisher->publish(msg);
}

/*
    This test is used to test the DDS communication between ROS2 and Zephyr
    It is used to validate the message conversion between ROS2 and Zephyr
    It is used to validate the sequence wrapper    
*/
int main(void) {
    printf("--------------------------------\n");
    printf("Starting DDS publisher\n");
    printf("--------------------------------\n");
    Node node("dds_test", node_stack, STACK_SIZE, timer_stack, STACK_SIZE);

    // Create a publisher for the test topic
    auto publisher = node.create_publisher<PoseStampedMsg>("test_pose", &geometry_msgs_msg_PoseStamped_desc);

    // Create a timer for the test topic
    node.create_timer(1000, timer_callback, static_cast<void*>(publisher.get()));

    printf("--------------------------------\n");
    printf("DDS publisher started\n");
    printf("--------------------------------\n");
    
    while(true) {
        sleep(1);
    }

    return 0;
}