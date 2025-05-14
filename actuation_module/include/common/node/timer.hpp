#ifndef COMMON__TIMER_HPP_
#define COMMON__TIMER_HPP_

// Standard Library
#include <string>
#include <cstring>
#include <pthread.h>
#include <time.h>
#include <signal.h>
#include <cerrno>

// Project headers
#include "common/logger/logger.hpp"
using namespace common::logger;

class Timer {
public:
    struct TimerHandlerData {
        void (*callback)(void*);
        void* arg;
        Timer* timer_instance;
    };

    Timer(const std::string& node_name, void* stack_area, size_t stack_size)
    : node_name_(node_name),
      timer_id_{0},
      timer_active_(false),
      ready_flag_(false),
      handler_data_(nullptr)
    {
        pthread_mutex_init(&mutex_, nullptr);
        pthread_attr_init(&timer_attr_);

        if (stack_area && stack_size > 0) {
            int ret = pthread_attr_setstack(&timer_attr_, stack_area, stack_size);
            if (ret != 0) {
                log_error("%s -> pthread_attr_setstack failed for timer thread: %s. Using default stack attributes.\n", node_name_.c_str(), strerror(ret));
                std::exit(1);
            }
        } else {
            log_warn("%s -> Timer stack area/size not provided or invalid. Timer thread will use default stack attributes.\n", node_name_.c_str());
            std::exit(1);
        }
    }

    ~Timer() {
        stop();
        pthread_mutex_destroy(&mutex_);
        pthread_attr_destroy(&timer_attr_);
    }

    bool start(uint32_t period_ms, void (*user_callback)(void*), void* user_arg = nullptr) {
        pthread_mutex_lock(&mutex_);
        if (timer_active_) {
            log_warn("%s -> Timer already active. Cannot create new timer.\n", node_name_.c_str());
            pthread_mutex_unlock(&mutex_);
            return false;
        }

        // Delete old data if any
        if (handler_data_) {
            delete handler_data_;
            handler_data_ = nullptr;
        }
        handler_data_ = new TimerHandlerData{user_callback, user_arg, this};
        
        struct sigevent sev;
        memset(&sev, 0, sizeof(struct sigevent));
        sev.sigev_notify = SIGEV_THREAD;
        sev.sigev_value.sival_ptr = handler_data_;
        sev.sigev_notify_function = Timer::static_timer_handler;
        sev.sigev_notify_attributes = &timer_attr_;

        if (timer_create(CLOCK_MONOTONIC, &sev, &timer_id_) < 0) {
            log_error("%s -> timer_create failed: %s\n", node_name_.c_str(), strerror(errno));
            delete handler_data_;
            handler_data_ = nullptr;
            pthread_mutex_unlock(&mutex_);
            return false;
        }
        
        struct itimerspec its;
        its.it_value.tv_sec  = period_ms / 1000;
        its.it_value.tv_nsec = (period_ms % 1000) * 1000000;
        its.it_interval.tv_sec  = period_ms / 1000; // Recurring interval
        its.it_interval.tv_nsec = (period_ms % 1000) * 1000000; // Recurring interval
        
        if (timer_settime(timer_id_, 0, &its, nullptr) < 0) {
            log_error("%s -> timer_settime failed: %s\n", node_name_.c_str(), strerror(errno));
            timer_delete(timer_id_);
            delete handler_data_;
            handler_data_ = nullptr;
            pthread_mutex_unlock(&mutex_);
            return false;
        }

        timer_active_ = true;
        ready_flag_ = false;
        pthread_mutex_unlock(&mutex_);
        log_info("%s -> Timer created and set with period %u ms\n", node_name_.c_str(), period_ms);
        return true;
    }

    void stop() {
        bool was_active = false;
        pthread_mutex_lock(&mutex_);
        if (timer_active_) {
            timer_delete(timer_id_);
            timer_active_ = false;
            was_active = true;
            
            if (handler_data_) {
                delete handler_data_;
                handler_data_ = nullptr;
            }
            ready_flag_ = false;
        }
        pthread_mutex_unlock(&mutex_);

        if (was_active) {
            log_info("%s -> Timer stopped and data released.\n", node_name_.c_str());
        }
    }

    /**
     * @brief Checks if the timer has fired and its callback is ready to be executed.
     * This method is thread-safe.
     * @return true if the timer callback is ready, false otherwise.
     */
    bool is_ready() {
        pthread_mutex_lock(&mutex_);
        bool is_ready_flag = ready_flag_;
        pthread_mutex_unlock(&mutex_);
        return is_ready_flag;
    }

    /**
     * @brief Executes the timer callback if the timer is active and ready.
     * This method is thread-safe. It will reset the ready flag after execution.
     */
    void execute_callback() {
        void (*user_callback)(void*) = nullptr;
        void* user_arg = nullptr;
        bool should_run = false;

        pthread_mutex_lock(&mutex_);
        if (timer_active_ && ready_flag_ && handler_data_ && handler_data_->callback) {
            user_callback = handler_data_->callback;
            user_arg = handler_data_->arg;
            ready_flag_ = false;
            should_run = true;
        }
        pthread_mutex_unlock(&mutex_);

        if (should_run) {
            user_callback(user_arg);
        }
    }

private:
    std::string node_name_;
    timer_t timer_id_;
    bool timer_active_;
    pthread_attr_t timer_attr_;
    
    // Synchronization primitives
    pthread_mutex_t mutex_;
    bool ready_flag_;

    // Timer handler primitives
    TimerHandlerData* handler_data_;

    static void static_timer_handler(union sigval val) {
        TimerHandlerData* data = static_cast<TimerHandlerData*>(val.sival_ptr);
        if (data && data->timer_instance) {
            data->timer_instance->instance_timer_handler();
        } else {
            log_error("static_timer_handler: Received null data or timer_instance.\n");
        }
    }

    void instance_timer_handler() {
        pthread_mutex_lock(&mutex_);

        if (!timer_active_) {
            log_warn("%s -> Timer callback called while timer is not active.\n", node_name_.c_str());
            pthread_mutex_unlock(&mutex_);
            return;
        }
        
        if (ready_flag_) { log_warn("%s -> Timer overrun detected.\n", node_name_.c_str()); }
        ready_flag_ = true;
        pthread_mutex_unlock(&mutex_);
    }
};

#endif  // COMMON__TIMER_HPP_
