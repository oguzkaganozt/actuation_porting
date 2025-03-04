#ifndef ACTUATION_AUTOWARE_NODE_HPP
#define ACTUATION_AUTOWARE_NODE_HPP

#include <string>
#include <typeinfo>

#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>

#include "config.hpp"
#include "node/dds.hpp"

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
        
        // Initialize timer
        timer_callback_ = nullptr;
        timer_user_data_ = nullptr;
        timer_active_ = false;
        k_timer_init(&node_timer_, timer_handler_static, nullptr);
        
        // Store the node pointer in user data
        k_timer_user_data_set(&node_timer_, this);

        // Initialize DDS settings
        struct ddsi_config dds_cfg;
        init_config(dds_cfg);

        // Create a DDS domain
        dds_entity_t domain = dds_create_domain_with_rawconfig(DDS_DOMAIN_ACTUATION, &dds_cfg);
        if (domain < 0 && domain != DDS_RETCODE_PRECONDITION_NOT_MET) {
            printk("dds_create_domain_with_rawconfig: %s\n", dds_strretcode(-domain));
            k_panic();
        }

        // Create a DDS participant
        m_dds_participant = dds_create_participant (DDS_DOMAIN_ACTUATION, NULL, NULL);
        if (m_dds_participant < 0) {
            printk("dds_create_participant: %s\n", dds_strretcode(-m_dds_participant));
            k_panic();
        }

        // Reliable QoS
        m_dds_qos = dds_create_qos();
        dds_qset_reliability(m_dds_qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));
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
            dds_retcode_t rc = dds_delete (m_dds_participant);    /* Deleting the participant will delete all its children recursively as well. */
            if (rc != DDS_RETCODE_OK) {
                printk("dds_delete: %s\n", dds_strretcode(-rc));
                k_panic();
            }
            dds_delete_qos(m_dds_qos);
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
     * @brief Subscribe to a topic
     * @param desc Topic descriptor
     * @param name Topic name
     * @param callback Callback function
     * @param user_data User data to pass to the callback DEFAULT: nullptr
     */
    template<typename M>
    bool create_subscription(const dds_topic_descriptor_t * desc, const char* name, void (*callback)(void*), void* user_data=nullptr) {
        dds_entity_t topic = dds_create_topic(m_dds_participant, desc, name, NULL, NULL);
        if (topic < 0) {
            printk("dds_create_topic (%s): %s\n", name, dds_strretcode(-topic));
            return false;
        }

        // Create a listener
        dds_listener_t * listener = dds_create_listener(m_dds_participant);
        dds_lset_data_available(listener, callback);

        dds_entity_t reader = dds_create_reader(m_dds_participant, topic, m_dds_qos, listener);
        if (reader < 0) {
            printk("dds_create_reader (%s): %s\n", name, dds_strretcode(-reader));
            return false;
        }

        // dds_delete_listener(listener); // TODO: Can we delete the listener as it is copied into the reader ?
        return true;
    }
    
    /**
     * @brief Publish a message to a topic
     * @param topic Topic name
     * @param message Message type
     * @return int 0 on success, negative error code on failure
     */
    template<typename M>
    bool create_publisher(const dds_topic_descriptor_t * desc, const char* name, const M& message) {
        dds_entity_t topic = dds_create_topic(m_dds_participant, desc, name, NULL, NULL);
        if (topic < 0) {
            printk("dds_create_topic (%s): %s\n", name, dds_strretcode(-topic));
            return false;
        }

        dds_entity_t writer = dds_create_writer(m_dds_participant, topic, m_dds_qos, NULL);
        if (writer < 0) {
            printk("dds_create_writer (%s): %s\n", name, dds_strretcode(-writer));
            return false;
        }

        dds_write(writer, &message);
        return true;
    }

    /**
     * @brief Create a timer that calls a function at regular intervals
     * @param interval_ms Interval in milliseconds
     * @param callback Function to call
     * @param user_data User data to pass to the callback DEFAULT: nullptr
     * @return bool true on success, false on failure
     */
    bool create_timer(uint32_t interval_ms, void (*callback)(void*), void* user_data=nullptr) {
        // Check if timer is already active
        if (timer_active_) {
            printk("Timer already active for node %s\n", name_.c_str());
            return false;
        }
        
        // Store callback info
        timer_callback_ = callback;
        timer_user_data_ = user_data;
        
        // Start timer
        k_timer_start(&node_timer_, K_MSEC(interval_ms), K_MSEC(interval_ms));
        timer_active_ = true;
        
        return true;
    }
    
    /**
     * @brief Cancel the timer
     * @return bool true on success, false if no timer was active
     */
    bool cancel_timer() {
        if (!timer_active_) {
            return false;
        }
        
        // Stop the timer
        k_timer_stop(&node_timer_);
        
        // Clear callback info
        timer_callback_ = nullptr;
        timer_user_data_ = nullptr;
        timer_active_ = false;
        
        return true;
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
    
    // Static timer handler that redirects to instance method
    static void timer_handler_static(struct k_timer* timer) {
        Node<T>* node = static_cast<Node<T>*>(k_timer_user_data_get(timer));
        if (node) {
            node->timer_handler();
        }
    }
    
    // Instance-specific timer handler
    void timer_handler() {
        if (timer_active_ && timer_callback_) {
            timer_callback_(timer_user_data_);
        }
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

    // DDS
    dds_entity_t m_dds_participant;
    dds_entity_t m_dds_writer;
    dds_entity_t m_dds_reader;
    dds_qos_t* m_dds_qos;
    static bool listener_static(dds_entity_t& reader) {
        dds_sample_info_t info;
        static T msg;

        dds_return_t rc = dds_take(reader, reinterpret_cast<void*>(&msg), &info, 1, 1);
        if (rc < 0) {
            printk("dds_take can't take msg: %s\n", dds_strretcode(-rc));
            return false;
        }
        else if (!info.valid_data) {
            printk("dds_take invalid data: %s\n", dds_strretcode(-rc));
            return false;
        }
        else
            return true;
    }

    // Single timer for the node
    struct k_timer node_timer_;
    void (*timer_callback_)(void*);
    void* timer_user_data_;
    bool timer_active_;
};

#endif  // ACTUATION_AUTOWARE_NODE_HPP
