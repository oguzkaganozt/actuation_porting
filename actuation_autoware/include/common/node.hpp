#ifndef NODE_HPP
#define NODE_HPP

#include <cstring>
#include <typeinfo>

#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/logging/log.h>

#include "common/dds.hpp"

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
    // Forward declaration of timer data
    struct NodeTimerData {
        void (*callback)(void*);
        void* user_data;
        bool in_use;
        struct k_timer timer;
    };

    /**
     * @brief Construct a new Node object
     * @param name Name of the node
     * @param stack_size Stack size for the node's thread
     * @param priority Thread priority
     */
    Node(const char* name, T& params, size_t stack_size = 2048, int priority = K_PRIO_PREEMPT(1))
        : name_(name)
        , parameters_(params)
        , stack_size_(stack_size)
        , priority_(priority)
        , running_(false)
    {
        // Allocate stack
        stack_ = k_malloc(stack_size_);
        
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
                    k_free(user_data);
                }
                k_timer_stop(&timer_data_[i].timer);
                timer_data_[i].in_use = false;
            }
        }
        
        // Free stack
        if (stack_) {
            k_free(stack_);
            stack_ = nullptr;
        }
    }
    
    /**
     * @brief Start the node's thread
     * @return int 0 on success, negative error code on failure
     */
    int start() {
        if (running_) {
            return 0; // Already running
        }
        
        if (!stack_) {
            return -ENOMEM;
        }
        
        // Create thread
        thread_id_ = k_thread_create(
            &thread_,
            static_cast<k_thread_stack_t*>(stack_),
            stack_size_,
            thread_entry,
            this, nullptr, nullptr,
            priority_,
            0,
            K_NO_WAIT);
        
        if (!thread_id_) {
            return -EAGAIN;
        }
        
        running_ = true;
        return 0;
    }
    
    /**
     * @brief Stop the node's thread
     */
    void stop() {
        if (running_ && thread_id_) {
            k_thread_abort(thread_id_);
            thread_id_ = nullptr;
            running_ = false;
        }
    }
    
    /**
     * @brief Get a modifiable reference to the parameter struct
     * 
     * @return T& Reference to the parameter struct for modification
     */
    T& parameters() {
        return parameters_;
    }

    /**
     * @brief Get the node name
     * @return const char* Node name
     */
    const char* get_name() const { 
        return name_; 
    }
    
    /**
     * @brief Create a timer that calls a function at regular intervals
     * @param interval_ms Interval in milliseconds
     * @param callback Function to call
     * @return int Timer ID or negative error code
     */
    int create_timer(uint32_t interval_ms, void (*callback)(void*), void* user_data) {
        // Find an available timer slot
        int timer_id = -1;
        for (int i = 0; i < MAX_TIMERS; i++) {
            if (!timer_data_[i].in_use) {
                timer_id = i;
                break;
            }
        }
        
        if (timer_id < 0) {
            return -ENOMEM; // No available timer slots
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
            return -ENOMEM;
        }
        data->node = this;
        data->timer_id = timer_id;
        
        k_timer_user_data_set(&timer_data_[timer_id].timer, data);
        k_timer_start(&timer_data_[timer_id].timer, K_MSEC(interval_ms), K_MSEC(interval_ms));
        
        return timer_id;
    }
    
    /**
     * @brief Cancel a timer
     * @param timer_id Timer ID returned by create_timer
     * @return int 0 on success, negative error code on failure
     */
    int cancel_timer(int timer_id) {
        if (timer_id < 0 || timer_id >= MAX_TIMERS || !timer_data_[timer_id].in_use) {
            return -EINVAL;
        }
        
        // Stop the timer
        k_timer_stop(&timer_data_[timer_id].timer);
        
        // Free user data
        void* user_data = k_timer_user_data_get(&timer_data_[timer_id].timer);
        if (user_data) {
            k_free(user_data);
        }
        
        // Mark as unused
        timer_data_[timer_id].in_use = false;
        timer_data_[timer_id].callback = nullptr;
        timer_data_[timer_id].user_data = nullptr;
        
        return 0;
    }

    /**
     * @brief Read configuration from a file
     */
    void conf_reader() {
        // Implementation of configuration reader
        // Can be customized to read from file and update parameters_
    }
    
private:
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
        if (timer_id >= 0 && timer_id < MAX_TIMERS && timer_data_[timer_id].in_use) {
            if (timer_data_[timer_id].callback) {
                timer_data_[timer_id].callback(timer_data_[timer_id].user_data);
            }
        }
    }
    
    static void thread_entry(void* p1, void* p2, void* p3) {
        Node<T>* node = static_cast<Node<T>*>(p1);
        if (node) {
            // Node thread main loop
            while (node->running_) {
                // Process DDS messages
                
                // Sleep to yield CPU
                k_msleep(1);
            }
        }
    }
    
    const char* name_;
    struct k_thread thread_;
    k_tid_t thread_id_;
    void* stack_;
    size_t stack_size_;
    int priority_;
    bool running_;

    // Parameter storage (struct-based)
    T parameters_;
    
    // Timer
    static constexpr int MAX_TIMERS = 8;
    NodeTimerData timer_data_[MAX_TIMERS];
};

#endif
