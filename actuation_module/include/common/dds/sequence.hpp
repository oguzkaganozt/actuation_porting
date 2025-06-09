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
#define MAX_SEQUENCE_SIZE 2048

/**
 * @brief Simplified DDS Sequence wrapper with vector-like interface
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
    
    T* sequence;  // Pointer to DDS sequence (either wrapped or allocated)
    
    // Ensure buffer has enough capacity, using DDS allocation
    bool ensure_capacity(size_t required_capacity) {
        if (!sequence) {
            log_error("Invalid sequence state - null sequence pointer");
            return false;
        }
        
        // Handle uninitialized sequence
        if (!sequence->_buffer) {
            sequence->_length = 0;
            sequence->_maximum = 0;
            sequence->_release = true;  // Mark for DDS cleanup
        }
        
        if (required_capacity <= static_cast<size_t>(sequence->_maximum)) {
            return true;  // Already have enough capacity
        }
        
        // Check maximum size limit
        if (required_capacity > MAX_SEQUENCE_SIZE) {
            log_error("Requested capacity %zu exceeds maximum allowed (%zu)", required_capacity, MAX_SEQUENCE_SIZE);
            return false;
        }
        
        // Calculate new capacity with reasonable growth
        size_t new_capacity = std::max(required_capacity, 
                                     std::max(static_cast<size_t>(sequence->_maximum) * 2, 
                                             static_cast<size_t>(DEFAULT_SEQUENCE_SIZE)));
        new_capacity = std::min(new_capacity, static_cast<size_t>(MAX_SEQUENCE_SIZE));
        
        using ElementType = typename std::remove_pointer<decltype(T::_buffer)>::type;
        
        // Use DDS allocation for new buffer
        ElementType* new_buffer = static_cast<ElementType*>(
            dds_alloc(new_capacity * sizeof(ElementType))
        );
        
        if (!new_buffer) {
            log_error("Failed to allocate DDS memory for sequence buffer");
            return false;
        }
        
        // Copy existing elements if any
        if (sequence->_buffer && sequence->_length > 0) {
            std::memcpy(new_buffer, sequence->_buffer, sequence->_length * sizeof(ElementType));
        }
        
        // Free old buffer using DDS if it was DDS-allocated
        if (sequence->_buffer && sequence->_release) {
            dds_free(sequence->_buffer);
        }
        
        // Update sequence with new buffer
        sequence->_buffer = new_buffer;
        sequence->_maximum = static_cast<uint32_t>(new_capacity);
        sequence->_release = true;  // Mark for DDS cleanup
        
        return true;
    }

public:
    using value_type = typename std::remove_pointer<decltype(T::_buffer)>::type;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using size_type = size_t;
    
    // Constructor 1: Wrap existing DDS sequence
    explicit Sequence(T& existing_seq) : sequence(&existing_seq) {}
    
    // Constructor 2: Create new DDS sequence with initial capacity
    explicit Sequence(size_type initial_capacity = DEFAULT_SEQUENCE_SIZE) : sequence(nullptr) {
        // Allocate DDS sequence structure
        sequence = static_cast<T*>(dds_alloc(sizeof(T)));
        if (!sequence) {
            log_error("Failed to allocate DDS sequence structure");
            return;
        }
        
        // Initialize sequence fields
        sequence->_length = 0;
        sequence->_maximum = 0;
        sequence->_release = true;  // DDS will clean up when freed
        sequence->_buffer = nullptr;
        
        // Allocate initial buffer if capacity requested
        if (initial_capacity > 0) {
            ensure_capacity(initial_capacity);
        }
    }
    
    // Disable copy operations to prevent accidental copying
    Sequence(const Sequence&) = delete;
    Sequence& operator=(const Sequence&) = delete;
    
    // Move operations
    Sequence(Sequence&& other) noexcept : sequence(other.sequence) {
        other.sequence = nullptr;
    }
    
    Sequence& operator=(Sequence&& other) noexcept {
        if (this != &other) {
            sequence = other.sequence;
            other.sequence = nullptr;
        }
        return *this;
    }
    
    // Destructor - DDS handles cleanup via _release flag
    ~Sequence() = default;
    
    // Basic operations
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
    
    // Data access with bounds checking
    reference at(size_type index) {
        if (!sequence || !sequence->_buffer) {
            throw std::runtime_error("Invalid sequence - null buffer");
        }
        if (index >= sequence->_length) {
            throw std::out_of_range("Index " + std::to_string(index) + 
                                  " out of range (size: " + std::to_string(sequence->_length) + ")");
        }
        return sequence->_buffer[index];
    }
    
    const_reference at(size_type index) const {
        if (!sequence || !sequence->_buffer) {
            throw std::runtime_error("Invalid sequence - null buffer");
        }
        if (index >= sequence->_length) {
            throw std::out_of_range("Index " + std::to_string(index) + 
                                  " out of range (size: " + std::to_string(sequence->_length) + ")");
        }
        return sequence->_buffer[index];
    }
    
    // Unchecked access operators
    reference operator[](size_type index) noexcept {
        return sequence->_buffer[index];
    }
    
    const_reference operator[](size_type index) const noexcept {
        return sequence->_buffer[index];
    }
    
    // Front/back access
    reference front() {
        if (empty()) {
            throw std::runtime_error("Cannot access front() of empty sequence");
        }
        return sequence->_buffer[0];
    }
    
    const_reference front() const {
        if (empty()) {
            throw std::runtime_error("Cannot access front() of empty sequence");
        }
        return sequence->_buffer[0];
    }
    
    reference back() {
        if (empty()) {
            throw std::runtime_error("Cannot access back() of empty sequence");
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    
    const_reference back() const {
        if (empty()) {
            throw std::runtime_error("Cannot access back() of empty sequence");
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    
    // Raw data access
    pointer data() noexcept {
        return sequence ? sequence->_buffer : nullptr;
    }
    
    const_pointer data() const noexcept {
        return sequence ? sequence->_buffer : nullptr;
    }
    
    // Iterators
    pointer begin() noexcept { return data(); }
    pointer end() noexcept { return data() + size(); }
    const_pointer begin() const noexcept { return data(); }
    const_pointer end() const noexcept { return data() + size(); }
    const_pointer cbegin() const noexcept { return begin(); }
    const_pointer cend() const noexcept { return end(); }
    
    // Capacity management
    bool reserve(size_type new_capacity) {
        return ensure_capacity(new_capacity);
    }
    
    // Modification operations
    void clear() noexcept {
        if (sequence) {
            sequence->_length = 0;
        }
    }
    
    bool push_back(const value_type& value) {
        if (!ensure_capacity(size() + 1)) {
            return false;
        }
        sequence->_buffer[sequence->_length++] = value;
        return true;
    }
    
    bool push_back(value_type&& value) {
        if (!ensure_capacity(size() + 1)) {
            return false;
        }
        sequence->_buffer[sequence->_length++] = std::move(value);
        return true;
    }
    
    void pop_back() {
        if (empty()) {
            throw std::runtime_error("Cannot pop_back() from empty sequence");
        }
        --sequence->_length;
    }
    
    bool resize(size_type new_size) {
        if (new_size > max_size()) {
            return false;
        }
        
        if (new_size > capacity() && !ensure_capacity(new_size)) {
            return false;
        }
        
        // Initialize new elements to default value if growing
        for (size_type i = sequence->_length; i < new_size; ++i) {
            sequence->_buffer[i] = value_type{};
        }
        
        sequence->_length = static_cast<uint32_t>(new_size);
        return true;
    }
    
    bool insert(size_type position, const value_type& value) {
        if (position > size()) {
            return false;
        }
        
        if (!ensure_capacity(size() + 1)) {
            return false;
        }
        
        // Shift elements after insertion point
        for (size_type i = size(); i > position; --i) {
            sequence->_buffer[i] = std::move(sequence->_buffer[i-1]);
        }
        
        sequence->_buffer[position] = value;
        ++sequence->_length;
        return true;
    }
    
    // Utility functions
    T* get_dds_sequence() noexcept { return sequence; }
    const T* get_dds_sequence() const noexcept { return sequence; }
    
    bool is_valid() const noexcept {
        return sequence != nullptr;
    }
    
    void debug_info(const char* name = "Sequence") const {
        if (!is_valid()) {
            log_info("%s: Invalid sequence", name);
            return;
        }
        log_info("%s: size=%zu, capacity=%zu, release=%s", 
                name, size(), capacity(), sequence->_release ? "true" : "false");
    }
};

// Helper function to create sequence wrapper
template<typename T>
Sequence<T> wrap(T& dds_sequence) {
    return Sequence<T>(dds_sequence);
}

#endif  // COMMON__SEQUENCE_HPP_