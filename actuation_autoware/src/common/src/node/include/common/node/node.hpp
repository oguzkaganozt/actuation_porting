#ifndef BASE_NODE_NODE_HPP
#define BASE_NODE_NODE_HPP

// C++ Standard Library
#include <cstddef>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <variant>
#include <optional>

// System headers
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>

// Project headers
#include "common/dds_impl/dds.hpp"

using param_type = std::variant<bool,
                              int64_t,
                              double,
                              std::string,
                              std::vector<bool>,
                              std::vector<int64_t>,
                              std::vector<double>,
                              std::vector<std::string>,
                              std::vector<uint8_t>>;

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
     * @brief Spin the node thread
     * @return 0 on success, negative value on failure
     */
    int spin();
    
    /**
     * @brief Stop the node thread
     */
    void stop();

    /** 
     * @brief Create a publisher for a topic
     * @param topic_name Topic name
     * @param topic_descriptor Topic descriptor 
     * @return Publisher<MessageT>* Pointer to the publisher
     */
    template<typename MessageT>
    std::unique_ptr<Publisher<MessageT>> create_publisher(const std::string& topic_name, 
                                                        const dds_topic_descriptor_t* topic_descriptor) {
        return dds_.create_publisher_dds<MessageT>(topic_name, topic_descriptor);
    }

    /**
     * @brief Create a subscription for a topic
     * @param topic_name Topic name
     * @param topic_descriptor Topic descriptor 
     * @param callback Callback function
     * @return bool true on success, false on failure
     */
    template<typename T>
    bool create_subscription(const std::string& topic_name, 
                           const dds_topic_descriptor_t* topic_descriptor, 
                           callback_subscriber<T> callback) {

        auto subscription = dds_.create_subscription_dds<T>(topic_name, topic_descriptor, callback);
        subscriptions_.push_back(std::move(subscription));
        
        return true;
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
        static_assert(std::is_constructible_v<param_type, ParamT>, 
                     "Parameter type must be one of the supported types in param_type variant");
        
        pthread_mutex_lock(&param_mutex_);
        
        auto it = parameters_map_.find(name);
        if (it == parameters_map_.end()) {
            parameters_map_[name] = default_value;
            pthread_mutex_unlock(&param_mutex_);
            return default_value;
        } else {
            try {
                ParamT value = std::get<ParamT>(it->second);
                pthread_mutex_unlock(&param_mutex_);
                return value;
            } catch (const std::bad_variant_access&) {
                printf("Warning: Parameter '%s' exists with different type. Not overwriting.\n", name.c_str());
                pthread_mutex_unlock(&param_mutex_);
                return default_value;
            }
        }
    }

    /**
     * @brief Get a parameter value by name
     * @param name Name of the parameter
     * @return std::optional<param_type> The parameter value if found, std::nullopt if not found
     */
    std::optional<param_type> get_parameter(const std::string& name) const {
        pthread_mutex_lock(&param_mutex_);
        
        auto it = parameters_map_.find(name);
        if (it != parameters_map_.end()) {
            param_type value = it->second;
            pthread_mutex_unlock(&param_mutex_);
            return value;
        }
        
        pthread_mutex_unlock(&param_mutex_);
        return std::nullopt;
    }

    /**
     * @brief Get a parameter value by name
     * @tparam ParamT Type of the parameter
     * @param name Name of the parameter
     * @return The parameter value or default-constructed value if not found
     */
    template<typename ParamT>
    ParamT get_parameter(const std::string& name) const {
        static_assert(std::is_constructible_v<param_type, ParamT>, 
                     "Parameter type must be one of the supported types in param_type variant");
        
        auto param = get_parameter(name);
        if (param) {
            try {
                return std::get<ParamT>(*param);
            } catch (const std::bad_variant_access&) {
                return ParamT{};
            }
        }
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
        static_assert(std::is_constructible_v<param_type, ParamT>, 
                     "Parameter type must be one of the supported types in param_type variant");
        
        pthread_mutex_lock(&param_mutex_);
        
        auto it = parameters_map_.find(name);
        if (it != parameters_map_.end()) {
            // Check if the new value's type matches the existing parameter's type
            if (it->second.index() == param_type(value).index()) {
                it->second = value;
                pthread_mutex_unlock(&param_mutex_);
                return true;
            }
            printf("Warning: Cannot set parameter '%s' with different type\n", name.c_str());
            pthread_mutex_unlock(&param_mutex_);
            return false;
        }
        
        pthread_mutex_unlock(&param_mutex_);
        return false;
    }

    /**
     * @brief Check if a parameter exists
     * @param name Name of the parameter
     * @return true if parameter exists, false otherwise
     */
    inline bool has_parameter(const std::string& name) const {
        pthread_mutex_lock(&param_mutex_);
        bool exists = parameters_map_.find(name) != parameters_map_.end();
        pthread_mutex_unlock(&param_mutex_);
        return exists;
    }

    /**
     * @brief Get the name of the node
     * @return std::string Name of the node
     */
    inline std::string get_name() const {
        return node_name_;
    }

    /**
     * @brief Create a timer
     * @param period_ms Period in milliseconds
     * @param callback Callback function
     * @return true if timer was created, false otherwise
     */
    bool create_timer(uint32_t period_ms, void (*callback)(Node*));

    /**
     * @brief Stop the timer
     */
    void stop_timer();

private:
    // Node
    std::string node_name_;

    // Parameter storage
    std::unordered_map<std::string, param_type> parameters_map_;
    mutable pthread_mutex_t param_mutex_;

    // Thread
    pthread_t thread_;
    bool thread_active_ = false;
    static void* thread_entry_(void* arg);
    void run_();

    // DDS
    DDS dds_;
    std::vector<std::unique_ptr<SubscriberBase>> subscriptions_;
    
    // Timer
    timer_t timer_id_;
    bool timer_active_ = false;
    static void timer_handler_(union sigval val);
    void (*timer_callback_)(Node*);
};

#endif  // BASE_NODE_NODE_HPP
