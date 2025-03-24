#include "common/node/node.hpp"
#include "common/clock/clock.hpp"
#include "common/sequence/sequence.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>
#include <pthread.h>
#include <time.h>
#include <vector>

// Msgs
#include "PoseStamped.h"
using PoseStampedMsg = geometry_msgs_msg_PoseStamped;

// Define a simple struct that mimics a DDS sequence for testing
struct TestSequence {
    int32_t* _buffer;
    uint32_t _length;
    uint32_t _maximum;
};

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

static void handle_timer(void* node) {
    pthread_mutex_lock(&g_state.mutex);
    g_state.timer.count++;
    printf("Timer #%d\n", g_state.timer.count);
    pthread_mutex_unlock(&g_state.mutex);
    Node* node_ptr = static_cast<Node*>(node);
    node_ptr->set_parameter("int_param", g_state.timer.count);
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
    node.create_timer(100, handle_timer, &node);  // 10Hz timer
    
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

void test_sequence() {
    
    // Test 1: Basic construction and operations with non-const sequence
    {   
        TEST_START(Sequence-Basic)
        // Create an empty sequence
        Sequence<TestSequence> seq(10);
        ASSERT_MSG(seq.size() == 0, "New sequence should be empty");
        ASSERT_MSG(seq.capacity() >= 10, "New sequence should have requested capacity");
        ASSERT_MSG(seq.empty(), "New sequence should be empty");
        
        // Add some elements
        for (int i = 0; i < 5; i++) {
            seq.push_back(i);
        }
        
        ASSERT_MSG(seq.size() == 5, "Sequence should have 5 elements");
        ASSERT_MSG(!seq.empty(), "Sequence should not be empty");
        
        // Test element access
        ASSERT_MSG(seq[0] == 0, "Element 0 should be 0");
        ASSERT_MSG(seq[4] == 4, "Element 4 should be 4");
        ASSERT_MSG(seq.front() == 0, "Front element should be 0");
        ASSERT_MSG(seq.back() == 4, "Back element should be 4");
        
        // Test reserve and capacity
        seq.reserve(20);
        ASSERT_MSG(seq.capacity() >= 20, "Capacity should be at least 20 after reserve");
        ASSERT_MSG(seq.size() == 5, "Size should still be 5 after reserve");
        
        // Test pop_back
        seq.pop_back();
        ASSERT_MSG(seq.size() == 4, "Size should be 4 after pop_back");
        ASSERT_MSG(seq.back() == 3, "Back element should now be 3");
        
        // Test clear
        seq.clear();
        ASSERT_MSG(seq.size() == 0, "Size should be 0 after clear");
        ASSERT_MSG(seq.empty(), "Sequence should be empty after clear");
        ASSERT_MSG(seq.capacity() >= 20, "Capacity should remain unchanged after clear");
        
        // Test resize
        seq.resize(8);
        ASSERT_MSG(seq.size() == 8, "Size should be 8 after resize");
        for (size_t i = 0; i < 8; i++) {
            ASSERT_MSG(seq[i] == 0, "New elements should be zero-initialized");
        }
        
        // Test insert
        seq.insert(3, 42);
        ASSERT_MSG(seq.size() == 9, "Size should be 9 after insert");
        ASSERT_MSG(seq[3] == 42, "Element at position 3 should be 42");
        
        // Test automatic reallocation
        for (int i = 0; i < 50; i++) {
            seq.push_back(i);
        }
        ASSERT_MSG(seq.size() == 59, "Size should be 59 after adding 50 more elements");
        ASSERT_MSG(seq.capacity() >= 59, "Capacity should be at least 59");

        TEST_END(Sequence-Basic)
    }
    
    // Test 2: Wrapping an existing sequence
    {
        TEST_START(Sequence-Wrap)

        TestSequence raw_seq = {};
        raw_seq._buffer = static_cast<int32_t*>(malloc(5 * sizeof(int32_t)));
        raw_seq._maximum = 5;
        raw_seq._length = 3;
        
        raw_seq._buffer[0] = 10;
        raw_seq._buffer[1] = 20;
        raw_seq._buffer[2] = 30;
        
        // Wrap the existing sequence
        Sequence<TestSequence> seq(raw_seq);
        
        ASSERT_MSG(seq.size() == 3, "Wrapped sequence should have size 3");
        ASSERT_MSG(seq.capacity() == 5, "Wrapped sequence should have capacity 5");
        ASSERT_MSG(seq[0] == 10, "Element 0 should be 10");
        ASSERT_MSG(seq[1] == 20, "Element 1 should be 20");
        ASSERT_MSG(seq[2] == 30, "Element 2 should be 30");
        
        // Modify the sequence
        seq.push_back(40);
        ASSERT_MSG(seq.size() == 4, "Size should be 4 after push_back");
        ASSERT_MSG(seq[3] == 40, "New element should be 40");
        
        // Verify the underlying raw sequence was modified
        ASSERT_MSG(raw_seq._length == 4, "Raw sequence length should be updated");
        ASSERT_MSG(raw_seq._buffer[3] == 40, "Raw sequence buffer should be updated");
        
        // Clean up
        free(raw_seq._buffer);

        TEST_END(Sequence-Wrap)
    }
    
    // Test 3: Const sequence operations
    {
        TEST_START(Sequence-Const)

        // Create a non-const sequence with data
        TestSequence raw_seq = {};
        raw_seq._buffer = static_cast<int32_t*>(malloc(5 * sizeof(int32_t)));
        raw_seq._maximum = 5;
        raw_seq._length = 3;
        
        raw_seq._buffer[0] = 100;
        raw_seq._buffer[1] = 200;
        raw_seq._buffer[2] = 300;
        
        // Wrap as const sequence
        Sequence<const TestSequence> const_seq = wrap_const(raw_seq);
        
        // Test read operations on const sequence
        ASSERT_MSG(const_seq.size() == 3, "Const sequence should have size 3");
        ASSERT_MSG(const_seq.capacity() == 5, "Const sequence should have capacity 5");
        ASSERT_MSG(const_seq[0] == 100, "Element 0 should be 100");
        ASSERT_MSG(const_seq[1] == 200, "Element 1 should be 200");
        ASSERT_MSG(const_seq[2] == 300, "Element 2 should be 300");
        
        // Test iterators
        int sum = 0;
        for (const auto& val : const_seq) {
            sum += val;
        }
        ASSERT_MSG(sum == 600, "Sum of elements should be 600");
        
        // Test creating a copy of the const sequence
        auto copy = const_seq.copy();
        ASSERT_MSG(copy.size() == 3, "Copied sequence should have size 3");
        ASSERT_MSG(copy[0] == 100, "Copied element 0 should be 100");
        ASSERT_MSG(copy[1] == 200, "Copied element 1 should be 200");
        ASSERT_MSG(copy[2] == 300, "Copied element 2 should be 300");
        
        // Can modify the copy
        copy.push_back(400);
        ASSERT_MSG(copy.size() == 4, "Modified copy should have size 4");
        ASSERT_MSG(const_seq.size() == 3, "Original const sequence should be unchanged");
        
        // Clean up
        free(raw_seq._buffer);

        TEST_END(Sequence-Const)
    }
    
    // Test 4: Move semantics
    {
        TEST_START(Sequence-Move)

        // Create a sequence with data
        Sequence<TestSequence> seq1(5);
        seq1.push_back(1);
        seq1.push_back(2);
        seq1.push_back(3);
        
        // Move construct
        Sequence<TestSequence> seq2(std::move(seq1));
        ASSERT_MSG(seq2.size() == 3, "Moved-to sequence should have size 3");
        ASSERT_MSG(seq2[0] == 1, "Moved-to sequence element 0 should be 1");
        ASSERT_MSG(seq2[1] == 2, "Moved-to sequence element 1 should be 2");
        ASSERT_MSG(seq2[2] == 3, "Moved-to sequence element 2 should be 3");
        
        // Source sequence should be empty/invalid after move
        ASSERT_MSG(!seq1.is_valid(), "Moved-from sequence should be invalid");
        
        // Create another sequence
        Sequence<TestSequence> seq3(2);
        seq3.push_back(4);
        seq3.push_back(5);
        
        // Move assign
        seq3 = std::move(seq2);
        ASSERT_MSG(seq3.size() == 3, "Move-assigned sequence should have size 3");
        ASSERT_MSG(seq3[0] == 1, "Move-assigned sequence element 0 should be 1");
        ASSERT_MSG(seq3[1] == 2, "Move-assigned sequence element 1 should be 2");
        ASSERT_MSG(seq3[2] == 3, "Move-assigned sequence element 2 should be 3");
        
        // Source sequence should be empty/invalid after move
        ASSERT_MSG(!seq2.is_valid(), "Move-assigned-from sequence should be invalid");

        TEST_END(Sequence-Move)
    }
    
    // Test 5: Large sequences and reallocation
    {
        TEST_START(Sequence-Large)

        Sequence<TestSequence> seq(10);
        
        // Add more elements than initial capacity
        for (int i = 0; i < 200; i++) {
            seq.push_back(i);
        }
        
        ASSERT_MSG(seq.size() == 200, "Sequence should have 200 elements");
        ASSERT_MSG(seq.capacity() >= 200, "Capacity should be at least 200");
        
        // Verify elements
        for (int i = 0; i < 200; i++) {
            ASSERT_MSG(seq[i] == i, "Element mismatch after reallocation");
        }

        TEST_END(Sequence-Large)
    }
    
    // Test 6: Testing merged implementation behavior
    {
        TEST_START(Sequence-Merged)

        // Create a non-const sequence
        TestSequence raw_seq = {};
        raw_seq._buffer = static_cast<int32_t*>(malloc(10 * sizeof(int32_t)));
        raw_seq._maximum = 10;
        raw_seq._length = 0;
        
        // Wrap as non-const
        {
            Sequence<TestSequence> seq(raw_seq);
            
            // Add some elements
            seq.push_back(10);
            seq.push_back(20);
            seq.push_back(30);
            
            ASSERT_MSG(seq.size() == 3, "Non-const sequence should have size 3");
            ASSERT_MSG(raw_seq._length == 3, "Underlying sequence should be updated");
        }
        
        // Now test using the same data but as const
        {
            // The TypeScript for const type - our implementation should handle this through SFINAE
            Sequence<const TestSequence> const_seq = wrap_const(raw_seq);
            
            // Read operations should work
            ASSERT_MSG(const_seq.size() == 3, "Const sequence should have size 3");
            ASSERT_MSG(const_seq[0] == 10, "Element 0 should be 10");
            ASSERT_MSG(const_seq[1] == 20, "Element 1 should be 20");
            ASSERT_MSG(const_seq[2] == 30, "Element 2 should be 30");
            
            // Test that we can iterate over const sequence
            std::vector<int> values;
            for (const auto& val : const_seq) {
                values.push_back(val);
            }
            ASSERT_MSG(values.size() == 3, "Should have 3 values from iteration");
            ASSERT_MSG(values[0] == 10, "First value should be 10");
            ASSERT_MSG(values[1] == 20, "Second value should be 20");
            ASSERT_MSG(values[2] == 30, "Third value should be 30");
            
            // Create a modifiable copy
            auto modifiable = const_seq.copy();
            modifiable.push_back(40);
            ASSERT_MSG(modifiable.size() == 4, "Copy should have 4 elements");
            ASSERT_MSG(const_seq.size() == 3, "Original should still have 3 elements");
            
            // Try to directly modify the const sequence - this won't compile
            // Uncomment to verify compile-time error:
            // const_seq.push_back(40);  // Should cause compile error
        }
        
        // Clean up
        free(raw_seq._buffer);

        TEST_END(Sequence-Merged)
    }
    
    // Test 7: Testing helper functions
    {
        TEST_START(Sequence-Helper)
        
        // Test wrap by reference
        {
            // Create a raw sequence
            TestSequence raw_seq = {};
            raw_seq._buffer = static_cast<int32_t*>(malloc(10 * sizeof(int32_t)));
            raw_seq._maximum = 10;
            raw_seq._length = 3;
            raw_seq._buffer[0] = 111;
            raw_seq._buffer[1] = 222;
            raw_seq._buffer[2] = 333;

            auto seq = wrap(raw_seq);
            ASSERT_MSG(seq.size() == 3, "Wrapped sequence should have size 3");
            ASSERT_MSG(seq[0] == 111, "Element 0 should be 111");
            
            // Modify through wrapper
            seq.push_back(444);
            ASSERT_MSG(raw_seq._length == 4, "Original sequence should be updated");
            ASSERT_MSG(raw_seq._buffer[3] == 444, "Original buffer should be updated");

            free(raw_seq._buffer);
        }
        
        // Test wrap_const by reference
        {
            // Create a raw sequence
            TestSequence raw_seq = {};
            raw_seq._buffer = static_cast<int32_t*>(malloc(10 * sizeof(int32_t)));
            raw_seq._maximum = 10;
            raw_seq._length = 3;
            raw_seq._buffer[0] = 111;
            raw_seq._buffer[1] = 222;
            raw_seq._buffer[2] = 333;

            auto seq = wrap_const(raw_seq);
            ASSERT_MSG(seq.size() == 3, "Const-wrapped sequence should have size 3");
            ASSERT_MSG(seq[0] == 111, "Element 0 should be 111");
            ASSERT_MSG(seq[2] == 333, "Element 2 should be 333");
            
            // Can't modify const wrapped sequence
            // Uncomment to test compile error:
            // seq.push_back(666);
            
            // But can make a modifiable copy
            auto copy = seq.copy();
            copy.push_back(666);
            ASSERT_MSG(copy.size() == 4, "Copy should have 4 elements");
            ASSERT_MSG(copy[3] == 666, "Element 3 of copy should be 666");
            ASSERT_MSG(seq.size() == 3, "Original const sequence should be unchanged");
            
            free(raw_seq._buffer);
        }

        TEST_END(Sequence-Helper)
    }
}

int main() {
    printf(COLOR_GREEN "=== Starting Node Test Suite ===\n" COLOR_RESET);
    Node node("test_node");
    
    test_parameters(node);
    test_clock_utils();
    test_timer_operations(node);
    test_thread_safety(node);
    test_dds_communication(node);
    test_sequence();

    printf(COLOR_GREEN "\n=== All Tests Passed ===\n" COLOR_RESET);
    exit(0);
}
