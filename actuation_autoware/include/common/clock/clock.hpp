#ifndef COMMON__CLOCK_HPP_
#define COMMON__CLOCK_HPP_

#include <chrono>
#include <cstdint>

//Msgs
#include "Time.h"

using TimeMsg = builtin_interfaces_msg_Time;
using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

class Clock {
public:
    Clock() = default;
    ~Clock() = default;

    TimePoint now() {
        return std::chrono::system_clock::now();
    }
};

// Convert ROS2 time to chrono time point
TimePoint toTimePoint(const TimeMsg& ros_time) {
    auto seconds = std::chrono::seconds(ros_time.sec);
    auto nanoseconds = std::chrono::nanoseconds(ros_time.nanosec);
    auto duration = seconds + nanoseconds;
    return TimePoint(duration);
}

// Convert chrono time point to ROS2 time
TimeMsg toRosTime(const TimePoint& chrono_time) {        
    auto duration = chrono_time.time_since_epoch();
    auto total_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
    
    TimeMsg ros_time;
    ros_time.sec = static_cast<int32_t>(total_nanoseconds.count() / 1000000000);
    ros_time.nanosec = static_cast<uint32_t>(total_nanoseconds.count() % 1000000000);
    
    return ros_time;
}

#endif  // COMMON__CLOCK_HPP_
