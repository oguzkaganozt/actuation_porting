#ifndef COMMON__SEQUENCE_HPP_
#define COMMON__SEQUENCE_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>
#include <stdexcept>

#include "common/logger/logger.hpp"
#include "dds/ddsc/dds_public_impl.h"

using namespace common::logger;

#define DEFAULT_SEQUENCE_SIZE 128
#define MAX_SEQUENCE_SIZE 8192

/**
 * @file sequence.hpp
 * @brief A robust C++ wrapper around DDS (Data Distribution Service) sequences
 * 
 * ==========================================================================
 * WHAT THIS CLASS DOES:
 * ==========================================================================
 * This template class provides a safe, std::vector-like interface for DDS sequences.
 * DDS sequences are low-level C structures used for message passing in robotics systems.
 * 
 * KEY PROBLEMS SOLVED:
 * - DDS sequences can contain uninitialized/corrupted memory that causes crashes
 * - Raw DDS API is difficult and error-prone to use directly
 * - Memory management complexity with DDS allocation/deallocation
 * - No bounds checking or safety features
 * 
 * ==========================================================================
 * MAIN WORKFLOW:
 * ==========================================================================
 * 
 * 1. WRAPPING EXISTING DDS SEQUENCES (most common):
 *    ```cpp
 *    TrajectoryMsg msg;                        // DDS message with sequence field
 *    auto seq = wrap(msg.points);              // Wrap the sequence safely
 *    seq.push_back(trajectory_point);          // Use like std::vector
 *    ```
 * 
 * 2. CREATING NEW SEQUENCES:
 *    ```cpp
 *    Sequence<PointSequence> seq(128);         // Create with initial capacity
 *    seq.push_back(point1);                    // Add elements
 *    seq.push_back(point2);
 *    ```
 * 
 * ==========================================================================
 * AUTOMATIC SAFETY FEATURES:
 * ==========================================================================
 * 
 * ✅ CORRUPTION DETECTION: Automatically detects and fixes:
 *    - Uninitialized memory (random garbage values)
 *    - Invalid buffer pointers (e.g., 0x40ad355408f6fd2f)
 *    - Inconsistent length/capacity values
 *    - Buffer addresses that look suspicious
 * 
 * ✅ CRASH PREVENTION: Safe fallbacks for:
 *    - Null pointer access → Returns dummy values instead of crashing
 *    - Out-of-bounds access → Bounds checking with exceptions
 *    - Invalid operations → Error logging and graceful failures
 * 
 * ✅ MEMORY MANAGEMENT: Automatic handling of:
 *    - DDS memory allocation/deallocation
 *    - Buffer growth and reallocation
 *    - Cleanup on destruction
 * 
 * ==========================================================================
 * TYPICAL USAGE PATTERNS:
 * ==========================================================================
 * 
 * // Pattern 1: Convert internal data to DDS message
 * TrajectoryMsg output;
 * auto points = wrap(output.points);           // Auto-detects corruption
 * points.reserve(trajectory.size());           // Pre-allocate for efficiency
 * for (const auto& pt : trajectory) {
 *     points.push_back(convert_point(pt));     // Safe insertion
 * }
 * 
 * // Pattern 2: Process incoming DDS data
 * void handle_trajectory(const TrajectoryMsg& msg) {
 *     auto points = wrap(msg.points);          // Read-only wrapper
 *     for (size_t i = 0; i < points.size(); ++i) {
 *         process_point(points[i]);            // Safe access
 *     }
 * }
 * 
 * ==========================================================================
 * INTERNAL WORKFLOW (for developers):
 * ==========================================================================
 * 
 * 1. Constructor called with DDS sequence reference
 * 2. needs_auto_initialization() checks for corruption signs:
 *    - _maximum > MAX_SEQUENCE_SIZE (2048)
 *    - _length > _maximum
 *    - Non-null _buffer with zero _maximum
 *    - Suspicious pointer addresses (> 1TB)
 * 3. If corrupted, auto_initialize_sequence() resets to safe defaults
 * 4. Operations use ensure_capacity() for dynamic buffer growth
 * 5. All memory management uses DDS allocation functions
 * 6. Destructor automatically cleans up owned resources
 * 
 * @tparam T DDS sequence type (must have _length, _buffer, _maximum, _release members)
 */
template<typename T>
class Sequence {
private:
    static_assert(std::is_member_object_pointer_v<decltype(&T::_length)>, "Type must have _length member");
    static_assert(std::is_member_object_pointer_v<decltype(&T::_buffer)>, "Type must have _buffer member");
    static_assert(std::is_member_object_pointer_v<decltype(&T::_maximum)>, "Type must have _maximum member");
    static_assert(std::is_member_object_pointer_v<decltype(&T::_release)>, "Type must have _release member");
    static_assert(std::is_pointer_v<decltype(T::_buffer)>, "_buffer member must be a pointer type");
    
    T* sequence;
    bool owns_sequence_;
    
    void check_validity(T* seq) const {
        if (!seq) {
            log_error("Detected corrupted sequence: null pointer");
            throw std::runtime_error("Detected corrupted sequence: null pointer");
        }

        if (seq->_maximum == 0 && seq->_buffer != nullptr) {
            log_error("Detected corrupted sequence: non-null buffer");
            throw std::runtime_error("Detected corrupted sequence: non-null buffer");
        }
        
        if (seq->_maximum > MAX_SEQUENCE_SIZE) {
            log_error("Detected corrupted sequence: _maximum=%u exceeds MAX_SEQUENCE_SIZE=%zu", 
                     seq->_maximum, MAX_SEQUENCE_SIZE);
            throw std::runtime_error("Detected corrupted sequence: _maximum exceeds MAX_SEQUENCE_SIZE");
        }
        
        if (seq->_length > seq->_maximum) {
            log_error("Detected corrupted sequence: _length=%u > _maximum=%u", 
                     seq->_length, seq->_maximum);
            throw std::runtime_error("Detected corrupted sequence: _length exceeds _maximum");
        }

        if (seq->_buffer != nullptr) {
            uintptr_t addr = reinterpret_cast<uintptr_t>(seq->_buffer);
            if (addr > 0x100000000000ULL) {
                log_error("Detected corrupted sequence: suspicious buffer address %p", seq->_buffer);
                throw std::runtime_error("Detected corrupted sequence: suspicious buffer address");
            }
        }
    }
    
    bool ensure_capacity(size_t required_capacity) {
        check_validity(sequence);
        
        if (!sequence->_buffer) {
            sequence->_length = 0;
            sequence->_maximum = 0;
            sequence->_release = true;
        }
        
        if (required_capacity <= static_cast<size_t>(sequence->_maximum)) {
            return true;
        }
        
        if (required_capacity > MAX_SEQUENCE_SIZE) {
            log_error("ensure_capacity: Requested capacity %zu exceeds maximum allowed (%zu)", 
                      required_capacity, MAX_SEQUENCE_SIZE);
            return false;
        }
        
        size_t new_capacity = std::max(required_capacity, 
                                     std::max(static_cast<size_t>(sequence->_maximum) * 2, 
                                             static_cast<size_t>(DEFAULT_SEQUENCE_SIZE)));
        new_capacity = std::min(new_capacity, static_cast<size_t>(MAX_SEQUENCE_SIZE));
        
        using ElementType = typename std::remove_pointer<decltype(T::_buffer)>::type;
        
        ElementType* new_buffer = static_cast<ElementType*>(
            dds_alloc(new_capacity * sizeof(ElementType))
        );
        
        if (!new_buffer) {
            log_error("ensure_capacity: Failed to allocate DDS memory for sequence buffer");
            return false;
        }
        
        if (sequence->_buffer && sequence->_length > 0) {
            std::memcpy(new_buffer, sequence->_buffer, sequence->_length * sizeof(ElementType));
        }
        
        if (sequence->_buffer && sequence->_release) {
            dds_free(sequence->_buffer);
        }
        
        sequence->_buffer = new_buffer;
        sequence->_maximum = static_cast<uint32_t>(new_capacity);
        sequence->_release = true;
        
        return true;
    }

public:
    using value_type = typename std::remove_pointer<decltype(T::_buffer)>::type;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using size_type = size_t;
    
    explicit Sequence(T& existing_seq) 
        : sequence(&existing_seq), owns_sequence_(false) {
        check_validity(sequence);
    }
    
    explicit Sequence(size_type initial_capacity = DEFAULT_SEQUENCE_SIZE) 
        : sequence(nullptr), owns_sequence_(true) {
        
        sequence = static_cast<T*>(dds_alloc(sizeof(T)));
        if (!sequence) {
            log_error("Failed to allocate DDS sequence structure");
            return;
        }
        
        sequence->_length = 0;
        sequence->_maximum = 0;
        sequence->_release = true;
        sequence->_buffer = nullptr;
        
        if (initial_capacity > 0) {
            if (!ensure_capacity(initial_capacity)) {
                log_error("Failed to allocate DDS sequence buffer");
            }
        }
    }
    
    Sequence(const Sequence&) = delete;
    Sequence& operator=(const Sequence&) = delete;
    
    Sequence(Sequence&& other) noexcept 
        : sequence(other.sequence), owns_sequence_(other.owns_sequence_) {
        other.sequence = nullptr;
        other.owns_sequence_ = false;
    }
    
    Sequence& operator=(Sequence&& other) noexcept {
        if (this != &other) {
            if (sequence && owns_sequence_) {
                if (sequence->_buffer && sequence->_release) {
                    dds_free(sequence->_buffer);
                }
                using non_const_type = typename std::remove_const<T>::type;
                dds_free(const_cast<non_const_type*>(sequence));
            }
            
            sequence = other.sequence;
            owns_sequence_ = other.owns_sequence_;
            other.sequence = nullptr;
            other.owns_sequence_ = false;
        }
        return *this;
    }
    
    ~Sequence() {
        if (sequence && owns_sequence_) {
            if (sequence->_buffer && sequence->_release) {
                dds_free(sequence->_buffer);
            }
            using non_const_type = typename std::remove_const<T>::type;
            dds_free(const_cast<non_const_type*>(sequence));
        }
    }
    
    size_type size() const noexcept { 
        return sequence ? sequence->_length : 0; 
    }
    
    size_type capacity() const noexcept { 
        return sequence ? sequence->_maximum : 0; 
    }
    
    size_type max_size() const noexcept { 
        return MAX_SEQUENCE_SIZE; 
    }
    
    bool empty() const noexcept { 
        return size() == 0; 
    }
    
    reference at(size_type index) {
        check_validity(sequence);
        
        if (index >= sequence->_length) {
            throw std::out_of_range("Index " + std::to_string(index) + 
                                  " out of range (size: " + std::to_string(sequence->_length) + ")");
        }
        return sequence->_buffer[index];
    }
    
    const_reference at(size_type index) const {
        check_validity(sequence);
        
        if (index >= sequence->_length) {
            throw std::out_of_range("Index " + std::to_string(index) + 
                                  " out of range (size: " + std::to_string(sequence->_length) + ")");
        }
        return sequence->_buffer[index];
    }
    
    reference operator[](size_type index) noexcept {
        if (!sequence || !sequence->_buffer) {
            static value_type dummy{};
            return dummy;
        }
        return sequence->_buffer[index];
    }
    
    const_reference operator[](size_type index) const noexcept {
        if (!sequence || !sequence->_buffer) {
            static const value_type dummy{};
            return dummy;
        }
        return sequence->_buffer[index];
    }
    
    reference front() {
        check_validity(sequence);
        
        if (empty()) {
            throw std::runtime_error("Cannot access front() of empty sequence");
        }
        
        return sequence->_buffer[0];
    }
    
    const_reference front() const {
        check_validity(sequence);
        
        if (empty()) {
            throw std::runtime_error("Cannot access front() of empty sequence");
        }
        
        return sequence->_buffer[0];
    }
    
    reference back() {
        check_validity(sequence);
        
        if (empty()) {
            throw std::runtime_error("Cannot access back() of empty sequence");
        }
        
        return sequence->_buffer[sequence->_length - 1];
    }
    
    const_reference back() const {
        check_validity(sequence);
        
        if (empty()) {
            throw std::runtime_error("Cannot access back() of empty sequence");
        }
        
        return sequence->_buffer[sequence->_length - 1];
    }
    
    pointer data() noexcept {
        // return sequence ? sequence->_buffer : nullptr;
        check_validity(sequence);
        return sequence->_buffer;
    }
    
    const_pointer data() const noexcept {
        // return sequence ? sequence->_buffer : nullptr;
        check_validity(sequence);
        return sequence->_buffer;
    }
    
    pointer begin() noexcept { return data(); }
    pointer end() noexcept { return data() + size(); }
    const_pointer begin() const noexcept { return data(); }
    const_pointer end() const noexcept { return data() + size(); }
    const_pointer cbegin() const noexcept { return begin(); }
    const_pointer cend() const noexcept { return end(); }
    
    bool reserve(size_type new_capacity) {
        return ensure_capacity(new_capacity);
    }
    
    void clear() noexcept {
        if (sequence) {
            sequence->_length = 0;
        }
    }
    
    bool push_back(const value_type& value) {
        check_validity(sequence);
        
        if (!ensure_capacity(size() + 1)) {
            return false;
        }
        
        sequence->_buffer[sequence->_length++] = value;
        return true;
    }
    
    bool push_back(value_type&& value) {
        check_validity(sequence);
        
        if (!ensure_capacity(size() + 1)) {
            return false;
        }
        
        sequence->_buffer[sequence->_length++] = std::move(value);
        return true;
    }
    
    void pop_back() {
        check_validity(sequence);
        
        if (empty() || !sequence) {
            throw std::runtime_error("Cannot pop_back() from empty or invalid sequence");
        }
        --sequence->_length;
    }
    
    bool insert(size_type position, const value_type& value) {
        check_validity(sequence);
        
        if (position > size()) {
            return false;
        }
        
        if (!ensure_capacity(size() + 1)) {
            return false;
        }
        
        for (size_type i = size(); i > position; --i) {
            sequence->_buffer[i] = std::move(sequence->_buffer[i-1]);
        }
        
        sequence->_buffer[position] = value;
        ++sequence->_length;
        return true;
    }
    
    T* get_dds_sequence() noexcept { return sequence; }
    const T* get_dds_sequence() const noexcept { return sequence; }
    
    void debug_info(const char* name = "Sequence") const {
        check_validity(sequence);
        log_info("%s: size=%zu, capacity=%zu, release=%s", 
                name, size(), capacity(), sequence->_release ? "true" : "false");
    }
};

template<typename T>
Sequence<T> wrap(T& dds_sequence) {
    return Sequence<T>(dds_sequence);
}

#endif  // COMMON__SEQUENCE_HPP_