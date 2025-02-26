#ifndef MESSAGE_ADAPTER_HPP_
#define MESSAGE_ADAPTER_HPP_

// Standard Library
#include <vector>
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
LOG_MODULE_DECLARE(message_adapter, LOG_LEVEL_DBG);

template<typename T>
class OwnedSequence {
private:
    using ElementType = typename std::remove_pointer<decltype(T::_buffer)>::type;
    std::vector<ElementType> data_vector;
    T sequence_struct;

public:
    // Default constructor
    OwnedSequence() {
        data_vector.reserve(128);
        sequence_struct._buffer = data_vector.data();
        sequence_struct._length = 0;
        sequence_struct._maximum = data_vector.capacity();
    }

    // Constructor with initial capacity
    explicit OwnedSequence(size_t initial_capacity) {
        data_vector.reserve(initial_capacity);
        sequence_struct._buffer = data_vector.data();
        sequence_struct._length = 0;
        sequence_struct._maximum = data_vector.capacity();
    }

    // Get the sequence struct
    T* get() {
        // Update pointers
        sequence_struct._buffer = data_vector.data();
        sequence_struct._maximum = data_vector.capacity();
        sequence_struct._length = data_vector.size();
        return &sequence_struct;
    }

    // Vector operations that update the sequence struct
    void push_back(const ElementType& value) {
        // Reserve more space if we're at capacity to reduce reallocations
        if (data_vector.size() == data_vector.capacity()) {
            data_vector.reserve(data_vector.capacity() * 1.5);
        }
        data_vector.push_back(value);
    }

    void resize(size_t new_size) {
        data_vector.resize(new_size);
    }

    void clear() {
        data_vector.clear();
    }
};

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
    std::unique_ptr<OwnedSequence<T>> owned_sequence;
    
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
        
        size_t new_capacity = required_capacity + 16;  // Growth strategy
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

    // Constructor that wraps an existing sequence (non-owning)
    explicit SequenceWrapper(T& seq) : sequence(&seq), owned_sequence(nullptr) {
        assert(sequence != nullptr && "Sequence pointer cannot be null");
    }
    
    // Default constructor that creates and owns a new sequence
    SequenceWrapper() : owned_sequence(std::make_unique<OwnedSequence<T>>()) {
        sequence = owned_sequence->get();
    }
    
    // Constructor that creates and owns a new sequence with initial capacity
    explicit SequenceWrapper(size_type initial_capacity) 
        : owned_sequence(std::make_unique<OwnedSequence<T>>(initial_capacity)) {
        sequence = owned_sequence->get();
    }
    
    // Destructor to clean up owned resources
    ~SequenceWrapper() {
        // No need to free resources when using OwnedSequence
        // OwnedSequence manages its own memory through std::vector
    }
    
    // Prevent copying to avoid double-free issues
    SequenceWrapper(const SequenceWrapper&) = delete;
    SequenceWrapper& operator=(const SequenceWrapper&) = delete;
    
    // Allow moving
    SequenceWrapper(SequenceWrapper&& other) noexcept
        : sequence(other.sequence), owned_sequence(std::move(other.owned_sequence)) {
        if (other.owned_sequence) {
            other.sequence = nullptr;
        }
    }
    
    SequenceWrapper& operator=(SequenceWrapper&& other) noexcept {
        if (this != &other) {
            sequence = other.sequence;
            owned_sequence = std::move(other.owned_sequence);
            
            if (other.owned_sequence) {
                other.sequence = nullptr;
            }
        }
        return *this;
    }
    
    // Get the underlying sequence
    T* get_sequence() {
        return sequence;
    }
    
    const T* get_sequence() const {
        return sequence;
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

    reference front() {
        if (empty()) {
            LOG_ERR("Cannot access front() of empty sequence");
            return sequence->_buffer[0];
        }
        return sequence->_buffer[0];
    }

    const_reference front() const {
        if (empty()) {
            LOG_ERR("Cannot access front() of empty sequence");
            return sequence->_buffer[0];
        }
        return sequence->_buffer[0];
    }

    reference back() {
        if (empty()) {
            LOG_ERR("Cannot access back() of empty sequence");
            return sequence->_buffer[0];
        }
        return sequence->_buffer[sequence->_length - 1];
    }

    const_reference back() const {
        if (empty()) {
            LOG_ERR("Cannot access back() of empty sequence");
            return sequence->_buffer[0];
        }
        return sequence->_buffer[sequence->_length - 1];
    }

    bool reserve(size_type new_cap) {
        return ensure_capacity(new_cap);
    }
    
    bool push_back(const value_type& value) {
        if (owned_sequence) {
            owned_sequence->push_back(value);
            sequence = owned_sequence->get(); // Update pointer in case of reallocation
            return true;
        } else {
            if (!ensure_capacity(sequence->_length + 1)) {
                return false;
            }
            
            sequence->_buffer[sequence->_length] = value;
            sequence->_length++;
            return true;
        }
    }
    
    bool pop_back() {
        if (sequence->_length > 0) {
            sequence->_length--;
            return true;
        }
        return false;
    }
    
    bool resize(size_type new_size) {
        if (owned_sequence) {
            owned_sequence->resize(new_size);
            sequence = owned_sequence->get();
            return true;
        } else {
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
    }
    
    void clear() {
        if (owned_sequence) {
            owned_sequence->clear();
            sequence = owned_sequence->get();
        } else {
            sequence->_length = 0;
        }
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

    // Element access with bounds checking
    reference at(size_type index) {
        if (index >= sequence->_length) {
            LOG_ERR("Index %zu out of range (size: %zu)", index, sequence->_length);
            return sequence->_buffer[0];
        }
        return sequence->_buffer[index];
    }

    const_reference at(size_type index) const {
        if (index >= sequence->_length) {
            LOG_ERR("Index %zu out of range (size: %zu)", index, sequence->_length);
            return sequence->_buffer[0];
        }
        return sequence->_buffer[index];
    }
};

// Helper function template
template<typename T>
inline SequenceWrapper<T> wrap_sequence(T& seq) {
    return SequenceWrapper<T>(seq);
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
