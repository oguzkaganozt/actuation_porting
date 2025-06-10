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
    bool owns_sequence_;
    bool is_constructed_successfully_;
    
    // Check if a DDS sequence needs initialization due to corruption or uninitialized state
    bool needs_auto_initialization(T* seq) const {
        if (!seq) return false;
        
        // Check for signs of uninitialized or corrupted sequence
        if (seq->_maximum > MAX_SEQUENCE_SIZE) {
            log_warn("Detected corrupted sequence: _maximum=%u exceeds MAX_SEQUENCE_SIZE=%zu", 
                     seq->_maximum, MAX_SEQUENCE_SIZE);
            return true;
        }
        
        if (seq->_length > seq->_maximum && seq->_maximum > 0) {
            log_warn("Detected corrupted sequence: _length=%u > _maximum=%u", 
                     seq->_length, seq->_maximum);
            return true;
        }
        
        if (seq->_buffer != nullptr && seq->_maximum == 0) {
            log_warn("Detected corrupted sequence: non-null buffer with zero maximum");
            return true;
        }
        
        // Check for obviously invalid buffer pointers (heuristic)
        if (seq->_buffer != nullptr) {
            uintptr_t addr = reinterpret_cast<uintptr_t>(seq->_buffer);
            // Check if address looks suspicious (very high addresses often indicate corruption)
            if (addr > 0x100000000000ULL) { // 1TB boundary as heuristic
                log_warn("Detected corrupted sequence: suspicious buffer address %p", seq->_buffer);
                return true;
            }
        }
        
        return false;
    }
    
    // Initialize a DDS sequence to safe defaults
    void auto_initialize_sequence(T* seq) {
        if (!seq) return;
        
        log_info("Auto-initializing DDS sequence to safe defaults");
        seq->_buffer = nullptr;
        seq->_length = 0;
        seq->_maximum = 0;
        seq->_release = true;
    }
    
    // Ensure buffer has enough capacity, using DDS allocation
    bool ensure_capacity(size_t required_capacity) {
        // log_debug("ensure_capacity: Starting, required=%zu, sequence=%p", required_capacity, sequence);
        
        if (!sequence) {
            log_error("ensure_capacity: Invalid sequence state - null sequence pointer");
            return false;
        }
        
        // log_debug("ensure_capacity: sequence valid, checking buffer state");
        // log_debug("ensure_capacity: _buffer=%p, _length=%u, _maximum=%u, _release=%s", 
        //           sequence->_buffer, sequence->_length, sequence->_maximum, 
        //           sequence->_release ? "true" : "false");
        
        // Handle uninitialized sequence
        if (!sequence->_buffer) {
            // log_debug("ensure_capacity: buffer is null, initializing sequence");
            sequence->_length = 0;
            sequence->_maximum = 0;
            sequence->_release = true;  // Mark for DDS cleanup
        }
        
        // log_debug("ensure_capacity: checking if current capacity %u >= required %zu", 
        //           sequence->_maximum, required_capacity);
        
        if (required_capacity <= static_cast<size_t>(sequence->_maximum)) {
            // log_debug("ensure_capacity: sufficient capacity available, returning true");
            return true;  // Already have enough capacity
        }
        
        // Check maximum size limit
        if (required_capacity > MAX_SEQUENCE_SIZE) {
            log_error("ensure_capacity: Requested capacity %zu exceeds maximum allowed (%zu)", 
                      required_capacity, MAX_SEQUENCE_SIZE);
            return false;
        }
        
        // Calculate new capacity with reasonable growth
        size_t new_capacity = std::max(required_capacity, 
                                     std::max(static_cast<size_t>(sequence->_maximum) * 2, 
                                             static_cast<size_t>(DEFAULT_SEQUENCE_SIZE)));
        new_capacity = std::min(new_capacity, static_cast<size_t>(MAX_SEQUENCE_SIZE));
        
        // log_debug("ensure_capacity: calculated new_capacity=%zu", new_capacity);
        
        using ElementType = typename std::remove_pointer<decltype(T::_buffer)>::type;
        
        // log_debug("ensure_capacity: allocating %zu bytes for buffer", new_capacity * sizeof(ElementType));
        
        // Use DDS allocation for new buffer
        ElementType* new_buffer = static_cast<ElementType*>(
            dds_alloc(new_capacity * sizeof(ElementType))
        );
        
        // log_debug("ensure_capacity: dds_alloc returned %p", new_buffer);
        
        if (!new_buffer) {
            log_error("ensure_capacity: Failed to allocate DDS memory for sequence buffer");
            return false;
        }
        
        // Copy existing elements if any
        if (sequence->_buffer && sequence->_length > 0) {
            // log_debug("ensure_capacity: copying %u existing elements from %p to %p", 
            //           sequence->_length, sequence->_buffer, new_buffer);
            std::memcpy(new_buffer, sequence->_buffer, sequence->_length * sizeof(ElementType));
        } else {
            // log_debug("ensure_capacity: no existing elements to copy");
        }
        
        // Free old buffer using DDS if it was DDS-allocated
        if (sequence->_buffer && sequence->_release) {
            // log_debug("ensure_capacity: freeing old buffer %p", sequence->_buffer);
            dds_free(sequence->_buffer);
        } else {
            // log_debug("ensure_capacity: not freeing old buffer (buffer=%p, release=%s)", 
            //           sequence->_buffer, sequence->_release ? "true" : "false");
        }
        
        // Update sequence with new buffer
        sequence->_buffer = new_buffer;
        sequence->_maximum = static_cast<uint32_t>(new_capacity);
        sequence->_release = true;  // Mark for DDS cleanup
        
        // log_debug("ensure_capacity: updated sequence - buffer=%p, maximum=%u, release=true", 
        //           sequence->_buffer, sequence->_maximum);
        
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
    explicit Sequence(T& existing_seq) 
        : sequence(&existing_seq), owns_sequence_(false), is_constructed_successfully_(true) {
        // log_debug("Sequence constructor (wrap): sequence=%p, buffer=%p, length=%u, maximum=%u", 
        //           sequence, sequence ? sequence->_buffer : nullptr,
        //           sequence ? sequence->_length : 0, sequence ? sequence->_maximum : 0);
        
        // Only auto-initialize non-const sequences to avoid compiler errors
        if constexpr (!std::is_const_v<T>) {
            // Automatically detect and fix corrupted/uninitialized DDS sequences
            if (sequence && needs_auto_initialization(sequence)) {
                auto_initialize_sequence(sequence);
            }
        } else {
            // For const sequences, we can only log warnings but not fix them
            if (sequence && needs_auto_initialization(sequence)) {
                log_warn("Const sequence appears corrupted but cannot be auto-initialized");
            }
        }
        
        // log_debug("After auto-initialization check: buffer=%p, length=%u, maximum=%u, release=%s",
        //           sequence ? sequence->_buffer : nullptr,
        //           sequence ? sequence->_length : 0, sequence ? sequence->_maximum : 0,
        //           sequence ? (sequence->_release ? "true" : "false") : "unknown");
    }
    
    // Constructor 2: Create new DDS sequence with initial capacity
    explicit Sequence(size_type initial_capacity = DEFAULT_SEQUENCE_SIZE) 
        : sequence(nullptr), owns_sequence_(true), is_constructed_successfully_(true) {
        // Allocate DDS sequence structure
        sequence = static_cast<T*>(dds_alloc(sizeof(T)));
        if (!sequence) {
            log_error("Failed to allocate DDS sequence structure");
            is_constructed_successfully_ = false;
            return;
        }
        
        // Initialize sequence fields
        sequence->_length = 0;
        sequence->_maximum = 0;
        sequence->_release = true;  // DDS will clean up when freed
        sequence->_buffer = nullptr;
        
        // Allocate initial buffer if capacity requested
        if (initial_capacity > 0) {
            if (!ensure_capacity(initial_capacity)) {
                log_error("Failed to allocate DDS sequence buffer");
                is_constructed_successfully_ = false;
            }
        }
    }
    
    // Disable copy operations to prevent accidental copying
    Sequence(const Sequence&) = delete;
    Sequence& operator=(const Sequence&) = delete;
    
    // Move operations
    Sequence(Sequence&& other) noexcept : sequence(other.sequence), owns_sequence_(other.owns_sequence_), is_constructed_successfully_(other.is_constructed_successfully_) {
        other.sequence = nullptr;
        other.owns_sequence_ = false;
        other.is_constructed_successfully_ = false;
    }
    
    Sequence& operator=(Sequence&& other) noexcept {
        if (this != &other) {
            // Clean up current resources
            if (sequence && owns_sequence_) {
                if (sequence->_buffer && sequence->_release) {
                    dds_free(sequence->_buffer);
                }
                // Remove const qualifier from type and cast to non-const pointer for dds_free
                using non_const_type = typename std::remove_const<T>::type;
                dds_free(const_cast<non_const_type*>(sequence));
            }
            
            // Transfer ownership
            sequence = other.sequence;
            owns_sequence_ = other.owns_sequence_;
            is_constructed_successfully_ = other.is_constructed_successfully_;
            other.sequence = nullptr;
            other.owns_sequence_ = false;
            other.is_constructed_successfully_ = false;
        }
        return *this;
    }
    
    // Destructor - DDS handles cleanup via _release flag
    ~Sequence() {
        if (sequence && owns_sequence_) {
            // Clean up buffer if we allocated it
            if (sequence->_buffer && sequence->_release) {
                dds_free(sequence->_buffer);
            }
            // Remove const qualifier from type and cast to non-const pointer for dds_free
            // This is safe because we own this sequence and allocated it with dds_alloc
            using non_const_type = typename std::remove_const<T>::type;
            dds_free(const_cast<non_const_type*>(sequence));
        }
    }
    
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
        // Add safety check to prevent crashes
        if (!sequence || !sequence->_buffer) {
            // Return reference to static dummy value to avoid crash
            static value_type dummy{};
            return dummy;
        }
        return sequence->_buffer[index];
    }
    
    const_reference operator[](size_type index) const noexcept {
        // Add safety check to prevent crashes
        if (!sequence || !sequence->_buffer) {
            // Return reference to static dummy value to avoid crash
            static const value_type dummy{};
            return dummy;
        }
        return sequence->_buffer[index];
    }
    
    // Front/back access
    reference front() {
        if (!sequence || !sequence->_buffer || empty()) {
            throw std::runtime_error("Cannot access front() of empty or invalid sequence");
        }
        return sequence->_buffer[0];
    }
    
    const_reference front() const {
        if (!sequence || !sequence->_buffer || empty()) {
            throw std::runtime_error("Cannot access front() of empty or invalid sequence");
        }
        return sequence->_buffer[0];
    }
    
    reference back() {
        if (!sequence || !sequence->_buffer || empty()) {
            throw std::runtime_error("Cannot access back() of empty or invalid sequence");
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    
    const_reference back() const {
        if (!sequence || !sequence->_buffer || empty()) {
            throw std::runtime_error("Cannot access back() of empty or invalid sequence");
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
        if (!sequence || !is_constructed_successfully_) {
            log_error("Cannot push_back to invalid sequence");
            return false;
        }
        
        if (!ensure_capacity(size() + 1)) {
            return false;
        }
        
        sequence->_buffer[sequence->_length++] = value;
        return true;
    }
    
    bool push_back(value_type&& value) {
        if (!sequence || !is_constructed_successfully_) {
            log_error("Cannot push_back to invalid sequence");
            return false;
        }
        
        if (!ensure_capacity(size() + 1)) {
            return false;
        }
        
        sequence->_buffer[sequence->_length++] = std::move(value);
        return true;
    }
    
    void pop_back() {
        if (empty() || !sequence) {
            throw std::runtime_error("Cannot pop_back() from empty or invalid sequence");
        }
        --sequence->_length;
    }
    
    bool resize(size_type new_size) {
        if (!sequence || !is_constructed_successfully_) {
            log_error("Cannot resize invalid sequence");
            return false;
        }
        
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
        return sequence != nullptr && is_constructed_successfully_;
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
    // log_debug("wrap function: wrapping DDS sequence at %p", &dds_sequence);
    // log_debug("wrap function: dds_sequence._buffer=%p, _length=%u, _maximum=%u, _release=%s", 
    //           dds_sequence._buffer, dds_sequence._length, dds_sequence._maximum, 
    //           dds_sequence._release ? "true" : "false");
    return Sequence<T>(dds_sequence);
}

#endif  // COMMON__SEQUENCE_HPP_