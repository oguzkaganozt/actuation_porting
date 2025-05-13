#ifndef COMMON__TIMER_HPP_
#define COMMON__TIMER_HPP_

// Standard Library
#include <string>
#include <cstring> // For memset, strerror
#include <pthread.h>
#include <time.h>
#include <signal.h>
#include <cerrno> // For errno

// Project headers
#include "common/logger/logger.hpp"
using namespace common::logger;
// Forward declaration to avoid circular dependency if Node needs to be known by Timer in detail
// class Node; // Not strictly needed if Node specific interactions are handled via passed-in data or callbacks not requiring Node type

class Timer {
public:
    struct TimerHandlerData {
        void (*callback)(void*);
        void* arg;
        Timer* timer_instance; // Points to the Timer object
    };

    Timer(const std::string& owner_name, void* stack_area, size_t stack_size)
    : owner_name_(owner_name),
      timer_active_(false),
      serve_signal_(false),
      ready_flag_(false),
      handler_data_(nullptr)
    {
        pthread_attr_init(&timer_attr_);
        if (stack_area && stack_size > 0) {
            pthread_attr_setstack(&timer_attr_, stack_area, stack_size);
        }
        // Else, use default stack

        pthread_mutex_init(&mutex_, nullptr);
        pthread_cond_init(&cond_, nullptr);
    }

    ~Timer() {
        stop(); // Ensure timer is stopped and resources are released

        pthread_mutex_destroy(&mutex_);
        pthread_cond_destroy(&cond_);
        pthread_attr_destroy(&timer_attr_);
    }

    bool create(uint32_t period_ms, void (*user_callback)(void*), void* user_arg = nullptr) {
        if (timer_active_) {
            log_warn("%s -> Timer already active. Cannot create new timer.\n", owner_name_.c_str());
            return false;
        }

        struct sigevent sev;
        memset(&sev, 0, sizeof(struct sigevent));

        // Delete old data if any (should not happen if logic is correct)
        if (handler_data_) {
            delete handler_data_;
            handler_data_ = nullptr;
        }
        handler_data_ = new TimerHandlerData{user_callback, user_arg, this};
        
        sev.sigev_notify = SIGEV_THREAD;
        sev.sigev_value.sival_ptr = handler_data_;
        sev.sigev_notify_function = Timer::static_timer_handler;
        sev.sigev_notify_attributes = &timer_attr_;

        if (timer_create(CLOCK_MONOTONIC, &sev, &timer_id_) < 0) {
            log_error("%s -> timer_create failed: %s\n", owner_name_.c_str(), strerror(errno));
            delete handler_data_;
            handler_data_ = nullptr;
            return false;
        }
        
        struct itimerspec its;
        its.it_value.tv_sec  = period_ms / 1000;
        its.it_value.tv_nsec = (period_ms % 1000) * 1000000;
        its.it_interval.tv_sec  = period_ms / 1000; // Recurring interval
        its.it_interval.tv_nsec = (period_ms % 1000) * 1000000; // Recurring interval
        
        if (timer_settime(timer_id_, 0, &its, nullptr) < 0) {
            log_error("%s -> timer_settime failed: %s\n", owner_name_.c_str(), strerror(errno));
            timer_delete(timer_id_);
            delete handler_data_;
            handler_data_ = nullptr;
            return false;
        }
        timer_active_ = true;
        log_info("%s -> Timer created and set with period %u ms\n", owner_name_.c_str(), period_ms);
        return true;
    }

    void stop() {
        if (timer_active_) {
            timer_delete(timer_id_); // This also disarms the timer
            timer_active_ = false;
            
            // Clean up handler data
            // Must lock mutex here to ensure thread safety with timer_handler
            pthread_mutex_lock(&mutex_);
            if (handler_data_) {
                delete handler_data_;
                handler_data_ = nullptr;
            }
            // If the timer thread is waiting on cond_, signal it to wake up and exit
            // This scenario is less likely if timer_delete works as expected (stops thread creation)
            // but good for robustness.
            ready_flag_ = false; // Prevent it from running callback
            serve_signal_ = true; // Allow it to pass the wait loop
            pthread_cond_signal(&cond_);
            pthread_mutex_unlock(&mutex_);

            log_info("%s -> Timer stopped and data released.\n", owner_name_.c_str());
        }
    }

    // --- Synchronization methods for Node's main thread ---
    void lock_mutex() { pthread_mutex_lock(&mutex_); }
    void unlock_mutex() { pthread_mutex_unlock(&mutex_); }
    
    bool get_ready_flag() const { return ready_flag_; }
    // void set_ready_flag(bool val) { ready_flag_ = val; } // Only timer handler should set this true

    bool get_serve_signal() const { return serve_signal_; }
    void set_serve_signal(bool val) { serve_signal_ = val; }

    void signal_cond() { pthread_cond_signal(&cond_); }
    void wait_cond() { pthread_cond_wait(&cond_, &mutex_); } // Mutex must be locked before calling

private:
    std::string owner_name_; // Name of the owning entity (e.g., Node name) for logging

    timer_t timer_id_;
    bool timer_active_;
    pthread_attr_t timer_attr_; // For the thread created by the timer
    
    // Synchronization primitives
    pthread_mutex_t mutex_;
    pthread_cond_t cond_;
    bool serve_signal_; // Set by Node to signal timer handler to proceed
    bool ready_flag_;   // Set by timer handler when it's ready to execute callback

    TimerHandlerData* handler_data_;

    static void static_timer_handler(union sigval val) {
        TimerHandlerData* data = static_cast<TimerHandlerData*>(val.sival_ptr);
        if (data && data->timer_instance) {
            data->timer_instance->instance_timer_handler();
        } else {
            // This case should ideally not happen if setup is correct
            log_error("static_timer_handler: Received null data or timer_instance.\n");
        }
    }

    void instance_timer_handler() {
        // This function is the actual timer thread function
        pthread_mutex_lock(&mutex_);

        if (ready_flag_) {
            // This indicates a timer overrun: the previous tick's callback hasn't been serviced yet.
            log_warn("%s -> Timer overrun detected.\n", owner_name_.c_str());
            // We might not want to execute the callback in this case, or queue it,
            // depending on desired behavior. For now, we just log and skip.
            // The main loop in Node should ideally process these faster than they arrive.
            pthread_mutex_unlock(&mutex_);
            return;
        }
        
        ready_flag_ = true; // Signal that the timer has fired and is ready for callback execution

        // Wait for the main application thread (Node) to signal that it's okay to proceed
        // This allows the Node to control when the timer callback actually runs,
        // ensuring it fits into the Node's execution cycle.
        while (!serve_signal_) {
            pthread_cond_wait(&cond_, &mutex_);
        }

        // Check if timer was stopped while waiting
        if (!timer_active_ || !handler_data_ || !handler_data_->callback) {
             log_debug("%s -> Timer stopped or callback/data became null while waiting for serve signal.\n", owner_name_.c_str());
             ready_flag_ = false; // Reset ready flag
             serve_signal_ = false; // Reset serve signal
             // No need to signal here, as the main thread is not waiting for *this* specific state after a stop.
             pthread_mutex_unlock(&mutex_);
             return;
        }

        // Execute the user-provided callback
        if (handler_data_ && handler_data_->callback) {
            // Unlock mutex before calling user callback to prevent deadlocks
            // if user callback tries to interact with this timer or the node in a way that re-locks.
            // User callback should be self-contained or use its own synchronization.
            void (*callback_to_run)(void*) = handler_data_->callback;
            void* arg_to_pass = handler_data_->arg;
            
            pthread_mutex_unlock(&mutex_);
            callback_to_run(arg_to_pass); // Execute callback without holding the lock
            pthread_mutex_lock(&mutex_); // Re-acquire lock to safely modify flags
        }

        // Reset flags for the next cycle
        ready_flag_ = false;
        serve_signal_ = false; // Node will set this to true again for the next serve

        pthread_cond_signal(&cond_); // Signal the Node's main thread that callback execution is complete
        pthread_mutex_unlock(&mutex_);
    }
};

#endif  // COMMON__TIMER_HPP_
