#include "common/node/node.hpp"
#include "common/clock/clock.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>
#include <pthread.h>
#include <time.h>

// Msgs
#include "PoseStamped.h"
using PoseStampedMsg = geometry_msgs_msg_PoseStamped;

// Test state with thread-safe access
struct TestState {
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    struct {
        bool received{false};
        double x, y, z;
    } pose;
    struct {
        bool triggered{false};
        int count{0};
    } timer;
    
    void reset() {
        pthread_mutex_lock(&mutex);
        pose = {false, 0, 0, 0};
        timer = {false, 0};
        pthread_mutex_unlock(&mutex);
    }
};

static TestState g_state;  // Global test state

// Add color definitions for terminal output
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_RESET   "\033[0m"

// Helper macros for test readability
#define TEST_START(name) \
    printf("\n=== Testing " #name " ===\n"); \
    g_state.reset();

#define TEST_END(name) \
    printf(COLOR_GREEN #name " tests passed" COLOR_RESET "\n");

#define ASSERT_MSG(condition, message) \
    do { \
        if (!(condition)) { \
            printf(COLOR_RED "Assertion failed: %s" COLOR_RESET "\n", message); \
            fflush(stdout); /* Ensure the reset code is output before abort */ \
            assert(false && message); \
        } \
    } while (0)

// Callback handlers
static void handle_pose(PoseStampedMsg& msg) {
    pthread_mutex_lock(&g_state.mutex);
    g_state.pose = {true, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
    printf("Received pose: (%.1f, %.1f, %.1f)\n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    pthread_mutex_unlock(&g_state.mutex);
}

static void handle_timer(Node* node) {
    pthread_mutex_lock(&g_state.mutex);
    g_state.timer.count++;
    printf("Timer #%d\n", g_state.timer.count);
    pthread_mutex_unlock(&g_state.mutex);
    node->set_parameter("int_param", g_state.timer.count);
}

// Test verification helpers
bool wait_for_event(std::function<bool()> check, int timeout_ms=2000) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
        if (check()) return true;
        struct timespec ts{0, 10'000'000};  // 10ms in nanoseconds
        nanosleep(&ts, nullptr);
    }
    return false;
}

void verify_pose_reception(double x, double y, double z) {
    pthread_mutex_lock(&g_state.mutex);
    ASSERT_MSG(g_state.pose.received, "Message should be received");
    ASSERT_MSG(g_state.pose.x == x, "X coordinate mismatch");
    ASSERT_MSG(g_state.pose.y == y, "Y coordinate mismatch");
    ASSERT_MSG(g_state.pose.z == z, "Z coordinate mismatch");
    pthread_mutex_unlock(&g_state.mutex);
}

// Test cases
void test_parameters(Node& node) {
    TEST_START(Parameters)
    
    // Proper parameter test sequence
    ASSERT_MSG(node.declare_parameter<bool>("enabled", true) == true, "Parameter declaration failed");
    ASSERT_MSG(node.has_parameter("enabled"), "Should have declared parameter");
    ASSERT_MSG(node.get_parameter<bool>("enabled") == true, "Parameter value retrieval failed");
    ASSERT_MSG(node.set_parameter<bool>("enabled", false), "Parameter set failed");
    ASSERT_MSG(node.get_parameter<bool>("enabled") == false, "Updated value retrieval failed");
    ASSERT_MSG(!node.has_parameter("disabled"), "Should not have undeclared parameter");
    ASSERT_MSG(!node.get_parameter<bool>("disabled"), "Should not be able to get undeclared parameter");
    
    TEST_END(Parameters)
}

void test_timer_operations(Node& node) {
    TEST_START(Timer Operations)
    
    node.spin();
    node.create_timer(100, handle_timer);  // 10Hz timer
    
    // Verify timer fires at least twice
    ASSERT_MSG(wait_for_event([] {
        pthread_mutex_lock(&g_state.mutex);
        bool result = g_state.timer.count >= 2;
        pthread_mutex_unlock(&g_state.mutex);
        return result;
    }), "Timer didn't trigger enough times");
    
    // Verify timer stops
    node.stop_timer();
    const int final_count = g_state.timer.count;
    sleep(1);
    ASSERT_MSG(g_state.timer.count == final_count, "Timer should stop");
    
    node.stop();
    TEST_END(Timer Operations)
}

void test_thread_safety(Node& node) {
    TEST_START(Thread Management)
    
    for (int i = 0; i < 3; i++) {
        ASSERT_MSG(node.spin() == 0, "Thread start failed");
        sleep(1);
        node.stop();
    }
    
    TEST_END(Thread Management)
}

void test_dds_communication(Node& node) {
    TEST_START(DDS Communication)
    
    node.spin();
    auto publisher = node.create_publisher<PoseStampedMsg>("test_pose", &geometry_msgs_msg_PoseStamped_desc);
    node.create_subscription<PoseStampedMsg>("test_pose", &geometry_msgs_msg_PoseStamped_desc, handle_pose);
    
    // Test message roundtrips
    for (int i = 0; i < 3; i++) {
        PoseStampedMsg msg = {};
        msg.pose.position = {i*1.0, i*2.0, i*3.0};
        ASSERT_MSG(publisher->publish(msg), "Publish should succeed");
        
        ASSERT_MSG(wait_for_event([] { 
            pthread_mutex_lock(&g_state.mutex);
            bool received = g_state.pose.received;
            pthread_mutex_unlock(&g_state.mutex);
            return received;
        }), "Message delivery timeout");
        
        verify_pose_reception(i*1.0, i*2.0, i*3.0);
    }
    
    node.stop();
    TEST_END(DDS Communication)
}

void test_clock_utils() {
    TEST_START(Clock Utilities)
    
    // Test Clock::now() returns valid time
    double now = Clock::now();
    ASSERT_MSG(now > 0, "Current time should be positive");
    
    // Test round-trip conversion: double -> ROS time -> double
    const double test_time = 1234.567;
    TimeMsg ros_time = Clock::toRosTime(test_time);
    
    // Verify conversion to ROS time
    ASSERT_MSG(ros_time.sec == 1234, "Seconds conversion error");
    ASSERT_MSG(ros_time.nanosec == 567000000, "Nanoseconds conversion error");
    
    // Verify round-trip conversion
    double converted_back = Clock::toDouble(ros_time);
    ASSERT_MSG(fabs(converted_back - test_time) < 1e-9, "Round-trip conversion error");
    
    // Test boundary/edge cases
    const double zero_time = 0.0;
    TimeMsg zero_ros_time = Clock::toRosTime(zero_time);
    ASSERT_MSG(zero_ros_time.sec == 0, "Zero seconds conversion error");
    ASSERT_MSG(zero_ros_time.nanosec == 0, "Zero nanoseconds conversion error");
    
    // Test very large time values
    const double large_time = 1e9;  // ~31.7 years
    TimeMsg large_ros_time = Clock::toRosTime(large_time);
    ASSERT_MSG(large_ros_time.sec == 1000000000, "Large time seconds conversion error");
    ASSERT_MSG(large_ros_time.nanosec == 0, "Large time nanoseconds conversion error");
    
    // Test very precise time values
    const double precise_time = 12.000000008;  // 12 seconds + 8 nanosecond
    TimeMsg precise_ros_time = Clock::toRosTime(precise_time);
    ASSERT_MSG(precise_ros_time.sec == 12, "Precise time seconds conversion error");
    ASSERT_MSG(precise_ros_time.nanosec == 8, "Precise time nanoseconds conversion error");
    
    TEST_END(Clock Utilities)
}

int main() {
    printf(COLOR_GREEN "=== Starting Node Test Suite ===\n" COLOR_RESET);
    Node node("test_node");
    
    test_clock_utils();
    test_parameters(node);
    test_timer_operations(node);
    test_thread_safety(node);
    test_dds_communication(node);

    printf(COLOR_GREEN "=== All Tests Passed ===\n" COLOR_RESET);
    exit(0);
}
