#ifndef COMMON__TIME_HPP_
#define COMMON__TIME_HPP_

#include <chrono>
#include <cstdint>

class Time {
public:
    Time() = default;
    ~Time() = default;

    static int64_t Now() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    }
};

#endif  // COMMON__TIME_HPP_
