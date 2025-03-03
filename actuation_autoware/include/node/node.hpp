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
        k_mutex_init(&timer_mutex);
        
        // Initialize timers
        for (int i = 0; i < MAX_TIMERS; i++) {
            timer_data_[i].in_use = false;
            timer_data_[i].callback = nullptr;
            timer_data_[i].user_data = nullptr;
        }

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
     * @return int Timer ID or negative error code
     */
    int create_timer(uint32_t interval_ms, void (*callback)(void*), void* user_data=nullptr) {
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
