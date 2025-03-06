#ifndef ACTUATION_AUTOWARE_NODE_HPP
#define ACTUATION_AUTOWARE_NODE_HPP

// PORT
#if defined(NATIVE_LINUX)
#include <linux/printk.h>
#elif defined(NATIVE_ZEPHYR)
#include <zephyr/sys/printk.h>
#endif

#include <string>
#include <any>
#include <unordered_map>
#include <pthread.h>
#include <signal.h>
#include <time.h>

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
    Node(const std::string& node_name);
    
    /**
     * @brief Destructor
     */
    ~Node();

    /** 
     * @brief Create a publisher for a topic
     * @param topic_name Topic name
     * @param topic_descriptor Topic descriptor 
     * @return Publisher<MessageT>* Pointer to the publisher
     */
    template<typename MessageT>
    Publisher<MessageT>* create_publisher(const std::string topic_name, const dds_topic_descriptor_t* topic_descriptor) {
        return dds_->create_publisher<MessageT>(topic_name, topic_descriptor);
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
        return dds_->create_subscription<T>(topic_name, topic_descriptor, callback);
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
        pthread_mutex_lock(&param_mutex);
        
        auto it = parameters_map_.find(name);
        if (it == parameters_map_.end()) {
            parameters_map_[name] = default_value;
            pthread_mutex_unlock(&param_mutex);
            return default_value;
        } else {
            try {
                ParamT value = std::any_cast<ParamT>(it->second);
                pthread_mutex_unlock(&param_mutex);
                return value;
            } catch (const std::bad_any_cast&) {
                printk("Warning: Parameter '%s' exists with different type. Not overwriting.\n", name.c_str());
                pthread_mutex_unlock(&param_mutex);
                return default_value;
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
        pthread_mutex_lock(&param_mutex);
        
        auto it = parameters_map_.find(name);
        if (it != parameters_map_.end()) {
            try {
                ParamT value = std::any_cast<ParamT>(it->second);
                pthread_mutex_unlock(&param_mutex);
                return value;
            } catch (const std::bad_any_cast&) {
                pthread_mutex_unlock(&param_mutex);
                return ParamT{};
            }
        }
        
        pthread_mutex_unlock(&param_mutex);
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
        pthread_mutex_lock(&param_mutex);
        
        auto it = parameters_map_.find(name);
        if (it != parameters_map_.end()) {
            it->second = value;
            pthread_mutex_unlock(&param_mutex);
            return true;
        }
        
        pthread_mutex_unlock(&param_mutex);
        return false;
    }

    /**
     * @brief Check if a parameter exists
     * @param name Name of the parameter
     * @return true if parameter exists, false otherwise
     */
    bool has_parameter(const std::string& name) const;

    /**
     * @brief Spin the node thread
     * @return 0 on success, negative value on failure
     */
    int spin();
    
    /**
     * @brief Stop the node thread
     */
    void stop();

    /**
     * @brief Create a timer
     * @param period_ms Period in milliseconds
     * @param callback Callback function
     * @return true if timer was created, false otherwise
     */
    bool create_timer(uint32_t period_ms, void (*callback)(void*));

    /**
     * @brief Cancel the timer
     */
    void cancel_timer();

    /**
     * @brief Get the name of the node
     * @return const char* Name of the node
     */
    const char* get_name() const;

private:
    // Node
    std::string node_name_;

    // Parameter storage
    std::unordered_map<std::string, std::any> parameters_map_;
    pthread_mutex_t param_mutex;

    // DDS
    DDS dds_;

    // Thread
    pthread_t thread_ {};
    static void* thread_entry(void* arg);

    // Timer
    bool timer_active_;
    timer_t* timer_id_;
    static void (*timer_callback_)(void*);

    // Make timer_signal_handler a static member function
    static void timer_signal_handler(int sig, siginfo_t *si, void *uc);
};

#endif  // ACTUATION_AUTOWARE_NODE_HPP
