#ifndef COMMON__NODE_HPP_
#define COMMON__NODE_HPP_

// C++ Standard Library
#include <cstddef>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <variant>
#include <optional>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>

// Project headers
#include "common/dds/dds.hpp"
#include "common/logger/logger.hpp"
using namespace common::logger;

using param_type = std::variant<bool,
                              int,
                              int64_t,
                              double,
                              std::string,
                              std::vector<bool>,
                              std::vector<int>,
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
    Node(const std::string& node_name, void* stack_area, size_t stack_size, void* timer_stack_area, size_t timer_stack_size)
    : node_name_(node_name)
    , param_mutex_(PTHREAD_MUTEX_INITIALIZER)
    , dds_(node_name)
    {
        pthread_attr_init(&thread_attr_);
        pthread_attr_setstack(&thread_attr_, stack_area, stack_size);

        pthread_attr_init(&timer_attr_);
        pthread_attr_setstack(&timer_attr_, timer_stack_area, timer_stack_size);
    }
    
    /**
     * @brief Destructor
     */
    ~Node() {
        stop_timer();
        stop();
    }

    /**
     * @brief Spin the node thread
     * @return 0 on success, negative value on failure
     */
    int spin() {
        return pthread_create(&thread_, &thread_attr_, main_thread_entry_, this);
    }
    
    /**
     * @brief Stop the node thread
     */
    void stop() {
        pthread_cancel(thread_);
        pthread_join(thread_, nullptr);
    }

    /** 
     * @brief Create a publisher for a topic
     * @param topic_name Topic name
     * @param topic_descriptor Topic descriptor 
     * @return Publisher<MessageT>* Pointer to the publisher
     */
    template<typename MessageT>
    std::shared_ptr<Publisher<MessageT>> create_publisher(const std::string& topic_name, 
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
                           callback_subscriber<T> callback, void* arg) {

        auto subscription = dds_.create_subscription_dds<T>(topic_name, topic_descriptor, callback, arg);
        subscriptions_.push_back(subscription);
        
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
    ParamT declare_parameter(const std::string& name, const ParamT& default_value = ParamT{}) {
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
                log_warn("Warning: Parameter '%s' exists with different type. Not overwriting.\n", name.c_str());
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
                log_warn("%s -> get_parameter failed-1: %s\n", node_name_.c_str(), name.c_str());
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
            log_warn("%s -> Cannot set parameter '%s' with different type\n", node_name_.c_str(), name.c_str());
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

    struct handler_data_t {
        void (*callback)(void*);
        void* arg;
    };

    /**
     * @brief Create a timer
     * @param period_ms Period in milliseconds
     * @param callback Callback function
     * @return true if timer was created, false otherwise
     */
    bool create_timer(uint32_t period_ms, void (*callback)(void*), void* arg = nullptr) {
        if (timer_active_) {
            return false;
        }

        struct sigevent sev;
        memset(&sev, 0, sizeof(struct sigevent));

        timer_handler_data_ = new handler_data_t{callback, arg};
        
        // TODO: Check if SIGEV_THREAD is supported in zephyr
        sev.sigev_notify = SIGEV_THREAD;
        sev.sigev_signo = SIGALRM;
        sev.sigev_value.sival_ptr = timer_handler_data_;
        sev.sigev_notify_function = timer_handler_;
        sev.sigev_notify_attributes = &timer_attr_;

        if (int ret = timer_create(CLOCK_MONOTONIC, &sev, &timer_id_); ret < 0) {
            log_error("%s -> timer creation failed: %s\n", node_name_.c_str(), strerror(errno));
            return false;
        }
        log_info("%s -> timer has been created\n", node_name_.c_str());
        
        struct itimerspec its;
        its.it_value.tv_sec  = period_ms / 1000;
        its.it_value.tv_nsec = (period_ms % 1000) * 1000000;
        its.it_interval.tv_sec  = period_ms / 1000;
        its.it_interval.tv_nsec = (period_ms % 1000) * 1000000;
        
        if (int ret = timer_settime(timer_id_, 0, &its, nullptr); ret < 0) {
            log_error("%s -> timer settime failed: %s\n", node_name_.c_str(), strerror(errno));
            timer_delete(timer_id_);
            return false;
        }
        timer_active_ = true;

        log_info("%s -> timer has been set with period %d ms\n", node_name_.c_str(), period_ms);
        return true;
    }

    /**
     * @brief Stop the timer
     */
    void stop_timer() {
        if (timer_active_) {
            timer_delete(timer_id_);
            timer_active_ = false;
            delete timer_handler_data_;
            timer_handler_data_ = nullptr;
            log_info("%s -> Timer stopped and data released.\n", node_name_.c_str());
        }
    }

private:
    // Node
    std::string node_name_;

    // Parameter storage
    std::unordered_map<std::string, param_type> parameters_map_;
    mutable pthread_mutex_t param_mutex_;

    // Thread
    pthread_t thread_;
    pthread_attr_t thread_attr_;
    pthread_attr_t timer_attr_;

    static void* thread_entry_(void* arg) {
        Node* node = static_cast<Node*>(arg);
        
        // TODO: Implement the node logic here
        // timer overruns are checked in timer_handler_
        // subscriptions are handled in the cyclonedds callback

        return nullptr;
    }

    // DDS
    DDS dds_;
    std::vector<std::shared_ptr<void>> subscriptions_;
    
    // Timer
    timer_t timer_id_;
    bool timer_active_ = false;
    handler_data_t* timer_handler_data_ = nullptr;

    static void timer_handler_(union sigval val) {
        handler_data_t* handler_data = static_cast<handler_data_t*>(val.sival_ptr);
        void (*callback)(void*) = handler_data->callback;
        callback(handler_data->arg);
    }
};

#endif  // COMMON__NODE_HPP_
