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

// Project headers
#include "common/dds/dds.hpp"
#include "common/node/timer.hpp"
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

        timer_ = std::make_unique<Timer>(node_name_, timer_stack_area, timer_stack_size);
    }
    
    /**
     * @brief Destructor
     */
    ~Node() {
        stop();

        pthread_mutex_destroy(&param_mutex_);
        pthread_attr_destroy(&thread_attr_);
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

    /**
     * @brief Create a timer
     * @param period_ms Period in milliseconds
     * @param callback Callback function
     * @return true if timer was created, false otherwise
     */
    bool create_timer(uint32_t period_ms, void (*callback)(void*), void* arg = nullptr) {
        if (!timer_) {
            log_error("%s -> Timer object not initialized!\n", node_name_.c_str());
            return false;
        }
        return timer_->create(period_ms, callback, arg);
    }

    /**
     * @brief Stop the timer
     */
    void stop_timer() {
        if (timer_) {
            timer_->stop();
            log_info("%s -> Timer stopped via Node request.\n", node_name_.c_str());
        } else {
            log_warn("%s -> Attempted to stop a non-initialized timer.\n", node_name_.c_str());
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

    static void* main_thread_entry_(void* arg) {
        Node* node = static_cast<Node*>(arg);
        
        while (1) {
            // log_debug("Main thread entry\n"); // Can be too verbose

            // Serve the timer if it's ready
            if (node->timer_) { // Check if timer object exists
                node->timer_->lock_mutex();
                if (node->timer_->get_ready_flag()) { // Timer has signaled it's ready
                    node->timer_->set_serve_signal(true); // Node signals timer to proceed
                    node->timer_->signal_cond();          // Wake up timer thread if it's waiting

                    // Wait for timer thread to finish its callback execution.
                    // The timer thread sets serve_signal_ to false when it's done and signals.
                    while (node->timer_->get_serve_signal()) {
                        node->timer_->wait_cond(); // Wait for timer to signal completion
                    }
                    // ready_flag is reset by the timer thread itself after callback execution
                }
                node->timer_->unlock_mutex(); // Unlock in all cases
            }

            // Send serve signal to the subscription threads
            // And wait for them to finish
            for (auto& subscription : node->subscriptions_) {
                subscription->execute();
            }
            // A small sleep might be useful here if the loop is too tight, depending on DDS behavior
            // For example: usleep(1000); // 1ms, if nothing else yields
        }
        return nullptr; // Should not be reached
    }

    // DDS
    DDS dds_;
    std::vector<std::shared_ptr<void>> subscriptions_;
    
    // Timer
    std::unique_ptr<Timer> timer_;
};

#endif  // COMMON__NODE_HPP_
