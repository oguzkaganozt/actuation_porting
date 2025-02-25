#ifndef MESSAGE_ADAPTER_HPP_
#define MESSAGE_ADAPTER_HPP_

// Standard Library
#include <vector>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>
#include <algorithm>
#include <memory>

// Zephyr
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
LOG_MODULE_REGISTER(message_adapter, LOG_LEVEL_DBG);

template<typename T>
class SequenceWrapper {
private:
    // Static assertions to verify sequence structure
    static_assert(std::is_member_object_pointer_v<decltype(&T::_length)>,
        "Type must have _length member");
    static_assert(std::is_member_object_pointer_v<decltype(&T::_buffer)>,
        "Type must have _buffer member");
    static_assert(std::is_member_object_pointer_v<decltype(&T::_maximum)>,
        "Type must have _maximum member");
    static_assert(std::is_trivially_copyable_v<typename std::remove_pointer<decltype(T::_buffer)>::type>,
        "Buffer element type must be trivially copyable");
    static_assert(std::is_pointer_v<decltype(T::_buffer)>,
        "_buffer member must be a pointer type");

    T* sequence;
    
    // Helper function to check and potentially grow the buffer
    bool ensure_capacity(size_t required_capacity) {
        if (required_capacity <= sequence->_maximum) {
            return true;  // Already have enough capacity
        }
        
        // Need to allocate more memory - use fixed growth to avoid excessive allocations
        // Add a cap to prevent excessive allocation attempts
        const size_t max_allowed_capacity = 1024;  // Adjust based on your system constraints
        if (required_capacity > max_allowed_capacity) {
            LOG_ERR("Requested capacity %zu exceeds maximum allowed (%zu)", 
                   required_capacity, max_allowed_capacity);
            return false;
        }
        
        size_t new_capacity = required_capacity + 8;  // More conservative growth strategy
        using ElementType = typename std::remove_pointer<decltype(T::_buffer)>::type;
        
        // Add size overflow check
        if (new_capacity > SIZE_MAX / sizeof(ElementType)) {
            LOG_ERR("Allocation size would overflow");
            return false;
        }
        
        ElementType* new_buffer = static_cast<ElementType*>(k_malloc(new_capacity * sizeof(ElementType)));
        if (!new_buffer) {
            LOG_ERR("Failed to allocate memory for sequence buffer");
            return false;
        }
        
        // Copy existing elements
        if (sequence->_buffer && sequence->_length > 0) {
            std::memcpy(new_buffer, sequence->_buffer, sequence->_length * sizeof(ElementType));
        }
        
        // Free old buffer
        if (sequence->_buffer) {
            k_free(sequence->_buffer);
        }
        
        // Update sequence
        sequence->_buffer = new_buffer;
        sequence->_maximum = new_capacity;
        
        return true;
    }

public:
    // Type definitions to make it more STL-compatible
    using value_type = typename std::remove_pointer<decltype(T::_buffer)>::type;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using reference = value_type&;
    using const_reference = const value_type&;
    using size_type = size_t;
    using iterator = value_type*;

    explicit SequenceWrapper(T& seq) : sequence(&seq) {
        assert(sequence != nullptr && "Sequence pointer cannot be null");
    }

    // Basic operations
    size_type size() const noexcept { 
        return static_cast<size_type>(sequence->_length); 
    }

    bool empty() const noexcept {
        return sequence->_length == 0;
    }

    size_type capacity() const noexcept {
        return sequence->_maximum;
    }

    // Element access - unchecked for performance
    reference operator[](size_type index) {
        return sequence->_buffer[index];
    }

    const_reference operator[](size_type index) const {
        return sequence->_buffer[index];
    }

    // Safe element access with error reporting
    bool at(size_type index, reference& out_value) {
        if (index >= sequence->_length) {
            LOG_ERR("Index %zu out of range (size: %zu)", index, sequence->_length);
            return false;
        }
        out_value = sequence->_buffer[index];
        return true;
    }

    // Iterator support - basic only
    pointer begin() { 
        return sequence->_buffer; 
    }
    
    pointer end() { 
        return sequence->_buffer + sequence->_length; 
    }

    const_pointer begin() const { 
        return sequence->_buffer; 
    }
    
    const_pointer end() const { 
        return sequence->_buffer + sequence->_length; 
    }

    // Data access
    pointer data() noexcept {
        return sequence->_buffer;
    }

    const_pointer data() const noexcept {
        return sequence->_buffer;
    }

    // Add direct front() method that returns a reference
    reference front() {
        assert(!empty() && "Cannot access front() of empty sequence");
        return sequence->_buffer[0];
    }

    // Add direct back() method that returns a reference
    reference back() {
        assert(!empty() && "Cannot access back() of empty sequence");
        return sequence->_buffer[sequence->_length - 1];
    }

    // Keep the existing safe versions
    bool front(reference& out_value) {
        if (empty()) {
            LOG_ERR("Cannot access front() of empty sequence");
            return false;
        }
        out_value = sequence->_buffer[0];
        return true;
    }

    bool back(reference& out_value) {
        if (empty()) {
            LOG_ERR("Cannot access back() of empty sequence");
            return false;
        }
        out_value = sequence->_buffer[sequence->_length - 1];
        return true;
    }

    // Capacity management
    bool reserve(size_type new_cap) {
        return ensure_capacity(new_cap);
    }
    
    // Add an element to the end
    bool push_back(const value_type& value) {
        if (!ensure_capacity(sequence->_length + 1)) {
            return false;
        }
        
        sequence->_buffer[sequence->_length] = value;
        sequence->_length++;
        return true;
    }
    
    // Remove the last element
    bool pop_back() {
        if (sequence->_length > 0) {
            sequence->_length--;
            return true;
        }
        return false;
    }
    
    // Change the size of the sequence
    bool resize(size_type new_size) {
        if (new_size > sequence->_maximum) {
            if (!ensure_capacity(new_size)) {
                return false;
            }
        }
        
        // If growing, initialize new elements to default value
        if (new_size > sequence->_length) {
            for (size_type i = sequence->_length; i < new_size; ++i) {
                sequence->_buffer[i] = value_type{};
            }
        }
        
        sequence->_length = new_size;
        return true;
    }
    
    // Remove all elements
    void clear() {
        sequence->_length = 0;
    }
    
    // Replace contents with count copies of value
    bool assign(size_type count, const value_type& value) {
        if (!ensure_capacity(count)) {
            return false;
        }
        
        for (size_type i = 0; i < count; ++i) {
            sequence->_buffer[i] = value;
        }
        
        sequence->_length = count;
        return true;
    }
    
    // Insert element at position
    bool insert_at(size_type position, const value_type& value) {
        if (position > sequence->_length) {
            LOG_ERR("Insert position %zu out of range (size: %zu)", position, sequence->_length);
            return false;
        }
        
        if (!ensure_capacity(sequence->_length + 1)) {
            LOG_ERR("Failed to allocate memory for insert operation");
            return false;
        }
        
        // Shift elements to make room using memmove for safety
        if (position < sequence->_length) {
            std::memmove(&sequence->_buffer[position + 1], 
                        &sequence->_buffer[position], 
                        (sequence->_length - position) * sizeof(value_type));
        }
        
        sequence->_buffer[position] = value;
        sequence->_length++;
        
        return true;
    }
    
    // Erase element at position
    bool erase_at(size_type position) {
        if (position >= sequence->_length) {
            LOG_ERR("Erase position %zu out of range (size: %zu)", position, sequence->_length);
            return false;
        }
        
        // Shift elements to fill the gap using memmove for safety
        if (position < sequence->_length - 1) {
            std::memmove(&sequence->_buffer[position], 
                        &sequence->_buffer[position + 1], 
                        (sequence->_length - position - 1) * sizeof(value_type));
        }
        
        sequence->_length--;
        return true;
    }
    
    // Vector conversion - simplified
    bool from_vector(const std::vector<value_type>& vec) {
        // Add size check
        if (vec.size() > SIZE_MAX / sizeof(value_type)) {
            LOG_ERR("Vector size would cause overflow");
            return false;
        }
        
        if (!ensure_capacity(vec.size())) {
            return false;
        }
        
        // Copy elements from vector to sequence
        if (!vec.empty()) {
            std::memcpy(sequence->_buffer, vec.data(), vec.size() * sizeof(value_type));
        }
        
        sequence->_length = vec.size();
        return true;
    }

    // Basic equality comparison
    bool equals(const SequenceWrapper& other) const {
        if (size() != other.size()) {
            return false;
        }
        
        for (size_type i = 0; i < size(); ++i) {
            if (!(sequence->_buffer[i] == other.sequence->_buffer[i])) {
                return false;
            }
        }
        
        return true;
    }

    // Add a method to check if the sequence has been properly initialized
    bool is_valid() const noexcept {
        return sequence != nullptr && sequence->_buffer != nullptr;
    }
};

// Helper function template
template<typename T>
inline SequenceWrapper<T> wrap_sequence(T& seq) {
    return SequenceWrapper<T>(seq);
}

// Helper function to create a sequence from a vector
template<typename T>
bool from_vector(T& seq, const std::vector<typename std::remove_pointer<decltype(T::_buffer)>::type>& vec) {
    return SequenceWrapper<T>(seq).from_vector(vec);
}

// Add logging function for debugging
template<typename T>
void log_sequence(const SequenceWrapper<T>& seq, const char* name = "SequenceWrapper") {
    LOG_INF("%s[size=%zu, capacity=%zu]", name, seq.size(), seq.capacity());
    
    if (!seq.empty()) {
        LOG_INF("Contents:");
        for (size_t i = 0; i < (seq.size() < 5 ? seq.size() : 5); ++i) {
            if constexpr (std::is_arithmetic_v<typename SequenceWrapper<T>::value_type>) {
                LOG_INF("  [%zu]: %d", i, static_cast<int>(seq[i]));
            } else {
                LOG_INF("  [%zu]: (complex element)", i);
            }
        }
        
        if (seq.size() > 5) {
            LOG_INF("  ... and %zu more elements", seq.size() - 5);
        }
    } else {
        LOG_INF("  (empty)");
    }
}

// Add a helper to safely initialize a sequence
template<typename T>
bool initialize_sequence(T& seq, size_t initial_capacity = 0) {
    // If already initialized or no capacity requested, just ensure length is 0
    if (seq._buffer != nullptr || initial_capacity == 0) {
        seq._length = 0;
        return true;
    }
    
    // Allocate new buffer
    using ElementType = typename std::remove_pointer<decltype(T::_buffer)>::type;
    seq._buffer = static_cast<ElementType*>(k_malloc(initial_capacity * sizeof(ElementType)));
    if (!seq._buffer) {
        return false;
    }
    
    seq._maximum = initial_capacity;
    seq._length = 0;
    return true;
}

// Add a helper to safely free a sequence
template<typename T>
void free_sequence(T& seq) {
    if (seq._buffer) {
        k_free(seq._buffer);
        seq._buffer = nullptr;
    }
    seq._maximum = 0;
    seq._length = 0;
}

#endif  // MESSAGE_ADAPTER_HPP_
