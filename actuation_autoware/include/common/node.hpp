#ifndef ACTUATION_AUTOWARE_NODE_HPP
#define ACTUATION_AUTOWARE_NODE_HPP

#include <string>
#include <typeinfo>

#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>

#include "common/dds.hpp"
#include "config.hpp"

/**
 * @brief Node class implementation to replicate ROS2 nodes within the Zephyr RTOS
 * 
 * This class provides a minimal implementation of a ROS2-like node for Zephyr RTOS,
 * supporting message-based communication through DDS. It includes thread management,
 * timers, and parameter storage.
 */
template<typename T>
class Node {
public:
    /**
     * @brief Construct a new Node object
     * @param name Name of the node
     * @param params Reference to node parameters struct
     * @param priority Thread priority
     * @param thread_sleep_ms Sleep time between iterations default 1ms
     */
    Node(const std::string& name, T& params, int thread_sleep_ms = 1, int priority = K_PRIO_PREEMPT(1))
        : name_(name)
        , parameters_(params)
        , thread_sleep_ms_(thread_sleep_ms)
        , priority_(priority)
    {
        // Initialize mutexes
        k_mutex_init(&param_mutex);
        k_mutex_init(&timer_mutex);
        
        // Initialize timers
        for (int i = 0; i < MAX_TIMERS; i++) {
            timer_data_[i].in_use = false;
            timer_data_[i].callback = nullptr;
            timer_data_[i].user_data = nullptr;
        }
    }
    
    /**
     * @brief Destructor
     */
    ~Node() {
        stop();
        
        // Cancel and cleanup all timers
        for (int i = 0; i < MAX_TIMERS; i++) {
            if (timer_data_[i].in_use) {
                void* user_data = k_timer_user_data_get(&timer_data_[i].timer);
                if (user_data) {
                    if(k_free(user_data)) {
                        printk("Failed to free timer user data for node %s\n", name_.c_str());
                        k_panic();
                    }
                }
                k_timer_stop(&timer_data_[i].timer);
                timer_data_[i].in_use = false;
            }
        }
    }
    
    /**
     * @brief Start the node's thread
     * @return int 0 on success, negative error code on failure
     */
    int start() {
        if (atomic_get(&running_)) {
            printk("Node %s already running\n", name_);
            return 0;
        }
        
        // Create thread
        thread_id_ = k_thread_create(
            &thread_,
            stack_,
            THREAD_STACK_SIZE,
            thread_entry,
            this, nullptr, nullptr,
            priority_,
            0,
            K_NO_WAIT);
        
        if (!thread_id_) {
            printk("Failed to create thread for node %s\n", name_.c_str());
            k_panic();
        }
        
        atomic_set(&running_, 1);
        return 0;
    }
    
    /**
     * @brief Stop the node's thread
     */
    void stop() {
        if (atomic_get(&running_) && thread_id_) {
            k_thread_abort(thread_id_);
            thread_id_ = nullptr;
            atomic_set(&running_, 0);
        }
    }

    /**
     * @brief Get the node name
     * @return const char* Node name
     */
    const char* get_name() const { 
        return name_; 
    }

    /**
     * @brief Set a specific parameter field (thread safe)
     * @tparam U Type of the parameter field
     * @param field Pointer to member of parameter struct
     * @param value New value to set
     */
    template<typename U>
    void set_parameter(U T::*field, const U& value) {
        k_mutex_lock(&param_mutex, K_FOREVER);
        parameters_.*field = value;
        k_mutex_unlock(&param_mutex);
    }
    
    /**
     * @brief Get a constant ref to node parameters
     * 
     * @return const T& Constant reference
     */
    T parameters() const {
        k_mutex_lock(&param_mutex, K_FOREVER);
        T params_copy = parameters_;
        k_mutex_unlock(&param_mutex);
        return params_copy;
    }
    
    /**
     * @brief Create a timer that calls a function at regular intervals
     * @param interval_ms Interval in milliseconds
     * @param callback Function to call
     * @return int Timer ID or negative error code
     */
    int create_timer(uint32_t interval_ms, void (*callback)(void*), void* user_data) {
        k_mutex_lock(&timer_mutex, K_FOREVER);
        
        // Find an available timer slot
        int timer_id = -1;
        for (int i = 0; i < MAX_TIMERS; i++) {
            if (!timer_data_[i].in_use) {
                timer_id = i;
                break;
            }
        }
        
        if (timer_id < 0) {
            k_mutex_unlock(&timer_mutex);
            printk("No available timer slots, Failed to create timer for node %s\n", name_.c_str());
            k_panic();
        }
        
        // Store callback info
        timer_data_[timer_id].callback = callback;
        timer_data_[timer_id].user_data = user_data;
        timer_data_[timer_id].in_use = true;
        
        // Initialize and start timer
        k_timer_init(&timer_data_[timer_id].timer, timer_handler_static, nullptr);
        
        // Store the node pointer and timer_id in user data
        struct timer_user_data* data = static_cast<struct timer_user_data*>(k_malloc(sizeof(struct timer_user_data)));
        if (!data) {
            timer_data_[timer_id].in_use = false;
            k_mutex_unlock(&timer_mutex);
            printk("Failed to allocate memory for timer user data for node %s\n", name_.c_str());
            k_panic();
        }
        data->node = this;
        data->timer_id = timer_id;
        
        k_timer_user_data_set(&timer_data_[timer_id].timer, data);
        k_timer_start(&timer_data_[timer_id].timer, K_MSEC(interval_ms), K_MSEC(interval_ms));
        
        k_mutex_unlock(&timer_mutex);
        return timer_id;
    }
    
    /**
     * @brief Cancel a timer
     * @param timer_id Timer ID returned by create_timer
     * @return int 0 on success, negative error code on failure
     */
    int cancel_timer(int timer_id) {
        k_mutex_lock(&timer_mutex, K_FOREVER);
        
        if (timer_id < 0 || timer_id >= MAX_TIMERS || !timer_data_[timer_id].in_use) {
            k_mutex_unlock(&timer_mutex);
            printk("Invalid timer ID, Failed to cancel timer for node %s\n", name_.c_str());
            k_panic();
        }
        
        // Stop the timer
        k_timer_stop(&timer_data_[timer_id].timer);
        
        // Free user data
        void* user_data = k_timer_user_data_get(&timer_data_[timer_id].timer);
        if (user_data) {
            if(k_free(user_data)) {
                printk("Failed to free timer user data for node %s\n", name_.c_str());
                k_panic();
            }
        }
        
        // Mark as unused
        timer_data_[timer_id].in_use = false;
        timer_data_[timer_id].callback = nullptr;
        timer_data_[timer_id].user_data = nullptr;
        
        k_mutex_unlock(&timer_mutex);
        return 0;
    }
    
private:
    // Thread entry point
    static void thread_entry(void* p1, void* p2, void* p3) {
        Node<T>* node = static_cast<Node<T>*>(p1);
        if (node) {
            while (atomic_get(&node->running_)) {
                // Process DDS messages
                k_msleep(node->thread_sleep_ms_);
            }
        }
    }

    // Structure to store in timer user data
    struct timer_user_data {
        Node<T>* node;
        int timer_id;
    };
    
    // Static timer handler that redirects to instance method
    static void timer_handler_static(struct k_timer* timer) {
        struct timer_user_data* data = static_cast<struct timer_user_data*>(k_timer_user_data_get(timer));
        if (data && data->node) {
            data->node->timer_handler(data->timer_id);
        }
    }
    
    // Instance-specific timer handler
    void timer_handler(int timer_id) {
        k_mutex_lock(&timer_mutex, K_FOREVER);
        if (timer_id >= 0 && timer_id < MAX_TIMERS && timer_data_[timer_id].in_use) {
            if (timer_data_[timer_id].callback) {
                void (*callback)(void*) = timer_data_[timer_id].callback;
                void* user_data = timer_data_[timer_id].user_data;
                k_mutex_unlock(&timer_mutex);
                callback(user_data);
                return;
            }
        }
        k_mutex_unlock(&timer_mutex);
    }
    
    std::string name_;
    struct k_thread thread_;
    k_tid_t thread_id_;
    K_THREAD_STACK_DEFINE(stack_, THREAD_STACK_SIZE);  // Initilize thread stack
    int priority_;
    atomic_t running_;

    // Parameter storage (struct-based)
    T parameters_;
    struct k_mutex param_mutex;

    // Timer
    struct NodeTimerData {
        void (*callback)(void*);
        void* user_data;
        bool in_use;
        struct k_timer timer;
    };
    static constexpr int MAX_TIMERS = 8;
    NodeTimerData timer_data_[MAX_TIMERS];
    struct k_mutex timer_mutex;
};

#endif  // ACTUATION_AUTOWARE_NODE_HPP
