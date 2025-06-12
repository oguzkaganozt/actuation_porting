#ifndef COMMON__TIMER_HPP_
#define COMMON__TIMER_HPP_

// Standard Library
#include <string>
#include <cstdint>

// Project headers
#include "common/logger/logger.hpp"
#include "common/clock/clock.hpp"
using namespace common::logger;

class Timer {
public:
    Timer(const std::string& node_name, void* stack_area = nullptr, size_t stack_size = 0)
    : node_name_(node_name),
      timer_active_(false),
      period_ms_(0),
      last_execution_time_(0.0),
      user_callback_(nullptr),
      user_arg_(nullptr)
    {
        // stack_area and stack_size parameters are kept for compatibility but not used
        // since we're not creating separate threads anymore
    }

    ~Timer() {
        stop();
    }

    bool start(uint32_t period_ms, void (*user_callback)(void*), void* user_arg = nullptr) {
        if (timer_active_) {
            log_warn("%s -> Timer already active. Cannot create new timer.\n", node_name_.c_str());
            return false;
        }

        if (!user_callback) {
            log_error("%s -> Timer callback cannot be null.\n", node_name_.c_str());
            return false;
        }

        period_ms_ = period_ms;
        user_callback_ = user_callback;
        user_arg_ = user_arg;
        last_execution_time_ = Clock::now();
        timer_active_ = true;

        log_info("%s -> Timer created and set with period %u ms\n", node_name_.c_str(), period_ms);
        return true;
    }

    void stop() {
        if (timer_active_) {
            timer_active_ = false;
            user_callback_ = nullptr;
            user_arg_ = nullptr;
            period_ms_ = 0;
            last_execution_time_ = 0.0;
            log_info("%s -> Timer stopped.\n", node_name_.c_str());
        }
    }

    /**
     * @brief Checks if the timer period has elapsed and the callback is ready to be executed.
     * @return true if the timer callback is ready, false otherwise.
     */
    bool is_ready() {
        if (!timer_active_ || !user_callback_) {
            return false;
        }

        double current_time = Clock::now();
        double elapsed_ms = (current_time - last_execution_time_) * 1000.0;
        
        return elapsed_ms >= static_cast<double>(period_ms_);
    }

    /**
     * @brief Executes the timer callback if the timer is active and ready.
     * This method will update the last execution time after calling the callback.
     */
    void execute_callback() {
        if (!timer_active_ || !user_callback_) {
            return;
        }

        // Check if it's time to execute
        if (!is_ready()) {
            return;
        }

        // Update execution time before calling callback to maintain consistent intervals
        last_execution_time_ = Clock::now();
        
        // Execute the user callback
        user_callback_(user_arg_);
    }

private:
    std::string node_name_;
    bool timer_active_;
    uint32_t period_ms_;
    double last_execution_time_;
    
    // Timer callback data
    void (*user_callback_)(void*);
    void* user_arg_;
};

#endif  // COMMON__TIMER_HPP_
