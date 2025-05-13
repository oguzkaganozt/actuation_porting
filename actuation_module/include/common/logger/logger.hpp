#ifndef COMMON__LOGGER_LOGGER_HPP_
#define COMMON__LOGGER_LOGGER_HPP_

#include <chrono>
#include <map>
#include <string>
#include <cstdio>
#include <cstdarg>
#include <pthread.h>

#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RESET "\033[0m"

namespace common::logger {

inline void vprint_color(const char * format, va_list args, const char * color) {
    // Get current time with milliseconds
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    char time_str[13]; // HH:MM:SS.mmm\0
    strftime(time_str, 9, "%H:%M:%S", localtime(&time_t_now));
    sprintf(time_str + 8, ".%03ld", ms.count());

    // Print message with time and color
    fprintf(stderr, "%s[%s] |", color, time_str);
    vfprintf(stderr, format, args);
    fprintf(stderr, "%s", COLOR_RESET);
}

inline void log_info(const char * format, ...) {
    #if CONFIG_LOG_LEVEL >= 1
    va_list args;
    va_start(args, format);
    vprint_color(format, args, COLOR_GREEN);
    va_end(args);
    #endif
}

inline void log_warn(const char * format, ...) {
    #if CONFIG_LOG_LEVEL >= 1
    va_list args;
    va_start(args, format);
    vprint_color(format, args, COLOR_YELLOW);
    va_end(args);
    #endif
}

inline void log_error(const char * format, ...) {
    #if CONFIG_LOG_LEVEL >= 1
    va_list args;
    va_start(args, format);
    vprint_color(format, args, COLOR_RED);
    va_end(args);
    #endif
}

inline void log_debug(const char * format, ...) {
    #if CONFIG_LOG_LEVEL >= 2
    va_list args;
    va_start(args, format);
    vprint_color(format, args, COLOR_RESET);
    va_end(args);
    #endif
}

inline void log_info_throttle(const char * msg, double interval_seconds=CONFIG_LOG_THROTTLE_RATE)
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

inline void log_warn_throttle(const char * msg, double interval_seconds=CONFIG_LOG_THROTTLE_RATE)
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
