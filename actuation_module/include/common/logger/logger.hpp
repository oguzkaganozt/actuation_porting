#ifndef COMMON__LOGGER_LOGGER_HPP_
#define COMMON__LOGGER_LOGGER_HPP_

#include <chrono>
#include <map>
#include <string>
#include <cstdio>
#include <pthread.h>

#ifndef CONFIG_LOG_THROTTLE_RATE
#define CONFIG_LOG_THROTTLE_RATE 3.0 // Default to 3 seconds if not defined in KConfig
#endif

namespace common::logger {

inline void info_throttle(const char * msg, double interval_seconds=CONFIG_LOG_THROTTLE_RATE)
{
    using clock = std::chrono::steady_clock;
    using time_point = clock::time_point;
    using duration = std::chrono::duration<double>;

    static pthread_mutex_t mutex_info = PTHREAD_MUTEX_INITIALIZER;
    static std::map<std::string, time_point> last_print_times;

    const std::string msg_str(msg);
    const auto now = clock::now();
    bool should_print = false;

    pthread_mutex_lock(&mutex_info);
    auto it = last_print_times.find(msg_str);
    if (it == last_print_times.end()) {
        should_print = true;
        last_print_times[msg_str] = now;
    } else {
        const duration time_since_last_print = now - it->second;
        if (time_since_last_print.count() >= interval_seconds) {
            should_print = true;
            it->second = now; // Update last print time
        }
    }
    pthread_mutex_unlock(&mutex_info);

    if (should_print) {
        printf("%s", msg);
    }
}

inline void warn_throttle(const char * msg, double interval_seconds=CONFIG_LOG_THROTTLE_RATE)
{
    using clock = std::chrono::steady_clock;
    using time_point = clock::time_point;
    using duration = std::chrono::duration<double>;

    static std::map<std::string, time_point> last_print_times_warn;
    static pthread_mutex_t mutex_warn = PTHREAD_MUTEX_INITIALIZER;

    const std::string msg_str(msg);
    const auto now = clock::now();
    bool should_print = false;

    pthread_mutex_lock(&mutex_warn);
    auto it = last_print_times_warn.find(msg_str);

    if (it == last_print_times_warn.end()) {
        should_print = true;
        last_print_times_warn[msg_str] = now;
    } else {
        const duration time_since_last_print = now - it->second;
        if (time_since_last_print.count() >= interval_seconds) {
            should_print = true;
            it->second = now; // Update last print time
        }
    }
    pthread_mutex_unlock(&mutex_warn);

    if (should_print) {
        fprintf(stderr, "%s", msg);
    }
}

} // namespace common::logger

#endif  // COMMON__LOGGER_LOGGER_HPP_
