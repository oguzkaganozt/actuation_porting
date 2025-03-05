#ifndef ACTUATION_AUTOWARE_NODE_HPP
#define ACTUATION_AUTOWARE_NODE_HPP

#include <string>
#include <typeinfo>
#include <unordered_map>
#include <any>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>

#include "node/dds.hpp"
#include "config.hpp"

/**
 * @brief Node base class implementation to replicate ROS2 nodes within the Zephyr RTOS
 * 
 * This class provides a minimal implementation of a ROS2-like node for Zephyr RTOS,
 * supporting message-based communication through DDS. It includes thread management,
 * timers, and parameter storage.
 */
class Node {
public:
    /**
     * @brief Construct a new Node object
     * @param node_name Name of the node
     */
    Node(const std::string& node_name)
        : node_name_(node_name)
        , running_(0)
    {
        // Initialize parameters mutex
        k_mutex_init(&param_mutex);
        
        // Initialize timer
        k_timer_init(&node_timer_, timer_expiry_handler, nullptr);

        // Initialize DDS
        dds_ = DDS(node_name);
    }
    
    /**
     * @brief Destructor
     */
    ~Node() {
        stop();
        
        // Stop timer if active
        if (timer_active_) {
            k_timer_stop(&node_timer_);
            timer_active_ = false;
        }
    }

    /** 
     * @brief Create a publisher for a topic
     * @param topic_name Topic name
     * @param topic_descriptor Topic descriptor
     * @return Publisher<MessageT>* Pointer to the publisher
     */
    template<typename MessageT>
    Publisher<MessageT>* create_publisher(const std::string topic_name, const dds_topic_descriptor_t* topic_descriptor) {
        return dds_.create_publisher<MessageT>(topic_name, topic_descriptor);
    }

    /**
     * @brief Create a subscription for a topic
     * @param topic_name Topic name
     * @param topic_descriptor Topic descriptor 
     * @param callback Callback function
     * @return bool true on success, false on failure
     */
    template<typename T>
    bool create_subscription(const std::string topic_name, const dds_topic_descriptor_t* topic_descriptor, void (*callback)(T*)) {
        return dds_.create_subscription<T>(topic_name, topic_descriptor, callback);
    }

    /**
     * @brief Declare a parameter with a name and default value
     * @tparam ParamT Type of the parameter
     * @param name Name of the parameter
     * @param default_value Default value for the parameter
     * @return The current value of the parameter (default if not previously set)
     */
    template<typename ParamT>
    ParamT declare_parameter(const std::string& name, const ParamT& default_value) {
        k_mutex_lock(&param_mutex, K_FOREVER);
        
        // Check if parameter already exists
        auto it = parameters_map_.find(name);
        if (it == parameters_map_.end()) {
            // Parameter doesn't exist, create it with default value
            parameters_map_[name] = default_value;
            k_mutex_unlock(&param_mutex);
            return default_value;
        } else {
            // Parameter exists, try to return its value
            try {
                ParamT value = std::any_cast<ParamT>(it->second);
                k_mutex_unlock(&param_mutex);
                return value;
            } catch (const std::bad_any_cast&) {
                // Type mismatch, log warning but keep original value
                printk("Warning: Parameter '%s' exists with different type. Not overwriting.\n", name.c_str());
                k_mutex_unlock(&param_mutex);
                return default_value;  // Return default value but don't change stored value
            }
        }
    }

    /**
     * @brief Get a parameter value by name
     * @tparam ParamT Type of the parameter
     * @param name Name of the parameter
     * @return The parameter value or default-constructed value if not found
     */
    template<typename ParamT>
    ParamT get_parameter(const std::string& name) const {
        k_mutex_lock(&param_mutex, K_FOREVER);
        
        auto it = parameters_map_.find(name);
        if (it != parameters_map_.end()) {
            try {
                ParamT value = std::any_cast<ParamT>(it->second);
                k_mutex_unlock(&param_mutex);
                return value;
            } catch (const std::bad_any_cast&) {
                // Type mismatch
                k_mutex_unlock(&param_mutex);
                return ParamT{};
            }
        }
        
        k_mutex_unlock(&param_mutex);
        return ParamT{};
    }

    /**
     * @brief Set a parameter value by name
     * @tparam ParamT Type of the parameter
     * @param name Name of the parameter
     * @param value Value to set
     * @return true if parameter was set, false if parameter doesn't exist
     */
    template<typename ParamT>
    bool set_parameter(const std::string& name, const ParamT& value) {
        k_mutex_lock(&param_mutex, K_FOREVER);
        
        auto it = parameters_map_.find(name);
        if (it != parameters_map_.end()) {
            it->second = value;
            k_mutex_unlock(&param_mutex);
            return true;
        }
        
        k_mutex_unlock(&param_mutex);
        return false;
    }

    /**
     * @brief Check if a parameter with the given name exists
     * @param name Name of the parameter to check
     * @return true if parameter exists, false otherwise
     */
    bool has_parameter(const std::string& name) const {
        k_mutex_lock(&param_mutex, K_FOREVER);
        bool exists = parameters_map_.find(name) != parameters_map_.end();
        k_mutex_unlock(&param_mutex);
        return exists;
    }

    /**
     * @brief Start the node background callback loop
     * @return int 0 on success, negative error code on failure
     */
    int spin(int priority = K_PRIO_PREEMPT(1), int thread_sleep_ms = 1) {
        if (atomic_get(&running_)) {
            printk("Node %s already running\n", node_name_.c_str());
            return 0;
        }

        // Create thread
        thread_id_ = k_thread_create(
            &thread_,
            stack_,
            THREAD_STACK_SIZE,
            thread_entry,
            this, nullptr, nullptr,
            priority,
            0,
            K_NO_WAIT);
        
        if (!thread_id_) {
            printk("Failed to create thread for node %s\n", node_name_.c_str());
            k_panic();
        }
        
        atomic_set(&running_, 1);
        thread_sleep_ms_ = thread_sleep_ms;
        return 0;
    }
    
    /**
     * @brief Stop the node background callback loop
     */
    void stop() {
        if (atomic_get(&running_) && thread_id_) {
            k_thread_abort(thread_id_);
            thread_id_ = nullptr;
            atomic_set(&running_, 0);
        }
    }

    /**
     * @brief Create a timer that calls a function at regular intervals
     * @param period_ms Period in milliseconds
     * @param callback Function to call
     * @return bool true on success, false on failure
     */
    bool create_timer(uint32_t period_ms, void (*callback)(void*)) {
        if (timer_active_) {
            printk("Timer already active for node %s\n", node_name_.c_str());
            return false;
        }
        
        k_timer_start(&node_timer_, K_MSEC(period_ms), K_MSEC(period_ms));
        timer_active_ = true;
        
        return true;
    }
    
    /**
     * @brief Stop the timer
     */
    void stop_timer() {
        if (timer_active_) {
            k_timer_stop(&node_timer_);
            timer_active_ = false;
        }
    }

    /**
     * @brief Get the node name
     * @return const char* Node name
     */
    const char* get_name() const { 
        return node_name_.c_str(); 
    }
    
private:
    // Thread entry point
    static void thread_entry(void* p1, void* p2, void* p3) {
        Node* node = static_cast<Node*>(p1);
        if (node) {
            while (atomic_get(&node->running_)) {
                // Process DDS messages
                k_msleep(node->thread_sleep_ms_);
            }
        }
    }

    // Node
    std::string node_name_;

    // Parameter storage (name-based)
    std::unordered_map<std::string, std::any> parameters_map_;
    mutable struct k_mutex param_mutex;

    // DDS
    DDS dds_;

    // Thread
    struct k_thread thread_;
    k_tid_t thread_id_;
    K_THREAD_STACK_DEFINE(stack_, THREAD_STACK_SIZE);  // Initilize thread stack
    int thread_sleep_ms_;
    atomic_t running_;

    // Timer
    struct k_timer node_timer_;
    void timer_expiry_handler(struct k_timer *timer_id) { k_work_submit(&timer_work_); }
    K_WORK_DEFINE(timer_work, timer_work_handler);
    void timer_work_handler(struct k_work *work) { ... }
    bool timer_active_;
};

#endif  // ACTUATION_AUTOWARE_NODE_HPP
