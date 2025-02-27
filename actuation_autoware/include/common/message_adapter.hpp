#ifndef MESSAGE_ADAPTER_HPP_
#define MESSAGE_ADAPTER_HPP_

#include <vector>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>
#include <algorithm>
#include <memory>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

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
class Sequence {
private:
    static_assert(std::is_member_object_pointer_v<decltype(&T::_length)>, "Type must have _length member");
    static_assert(std::is_member_object_pointer_v<decltype(&T::_buffer)>, "Type must have _buffer member");
    static_assert(std::is_member_object_pointer_v<decltype(&T::_maximum)>, "Type must have _maximum member");
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
            printk("Requested capacity %zu exceeds maximum allowed (%zu)\n", required_capacity, max_allowed_capacity);
            return false;
        }
        
        size_t new_capacity = required_capacity + 16;  // Growth strategy
        using ElementType = typename std::remove_pointer<decltype(T::_buffer)>::type;
        
        // Size overflow check
        if (new_capacity > SIZE_MAX / sizeof(ElementType)) {
            printk("Allocation size would overflow\n");
            return false;
        }
        
        ElementType* new_buffer = static_cast<ElementType*>(k_malloc(new_capacity * sizeof(ElementType)));
        if (!new_buffer) {
            printk("Failed to allocate memory for sequence buffer\n");
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
    using value_type = typename std::remove_pointer<decltype(T::_buffer)>::type;
    using reference = value_type&;
    using const_reference = const value_type&;
    using size_type = size_t;
    
    // Non-owning constructor
    explicit Sequence(T& seq) : sequence(&seq), owned_sequence(nullptr) {
        assert(sequence != nullptr && "Sequence pointer cannot be null");
    }
    
    // Owning constructors
    Sequence() : owned_sequence(std::make_unique<OwnedSequence<T>>()) {
        sequence = owned_sequence->get();
    }
    
    // Constructor that creates and owns a new sequence with initial capacity
    explicit Sequence(size_type initial_capacity) 
        : owned_sequence(std::make_unique<OwnedSequence<T>>(initial_capacity)) {
        sequence = owned_sequence->get();
    }
    
    // Prevent copying
    Sequence(const Sequence&) = delete;
    Sequence& operator=(const Sequence&) = delete;
    
    // Allow moving
    Sequence(Sequence&& other) noexcept
        : sequence(other.sequence), owned_sequence(std::move(other.owned_sequence)) {
        if (other.owned_sequence) {
            other.sequence = nullptr;
        }
    }
    
    Sequence& operator=(Sequence&& other) noexcept {
        if (this != &other) {
            sequence = other.sequence;
            owned_sequence = std::move(other.owned_sequence);
            
            if (other.owned_sequence) {
                other.sequence = nullptr;
            }
        }
        return *this;
    }
    
    T* get_sequence() { return sequence; }
    const T* get_sequence() const { return sequence; }

    // Basic vector-like operations
    size_type size() const noexcept { return sequence->_length; }
    bool empty() const noexcept { return sequence->_length == 0; }
    size_type capacity() const noexcept { return sequence->_maximum; }

    // Element access
    reference operator[](size_type index) { return sequence->_buffer[index]; }
    const_reference operator[](size_type index) const { return sequence->_buffer[index]; }

    reference at(size_type index) {
        if (index >= sequence->_length) {
            printk("Index %zu out of range (size: %zu)\n", index, sequence->_length);
            return sequence->_buffer[0]; // Return first element as fallback
        }
        return sequence->_buffer[index];
    }

    const_reference at(size_type index) const {
        if (index >= sequence->_length) {
            printk("Index %zu out of range (size: %zu)\n", index, sequence->_length);
            return sequence->_buffer[0]; // Return first element as fallback
        }
        return sequence->_buffer[index];
    }
    
    // Iterators
    value_type* begin() { return sequence->_buffer; }
    value_type* end() { return sequence->_buffer + sequence->_length; }
    const value_type* begin() const { return sequence->_buffer; }
    const value_type* end() const { return sequence->_buffer + sequence->_length; }
    
    // Data access
    value_type* data() noexcept { return sequence->_buffer; }
    const value_type* data() const noexcept { return sequence->_buffer; }
    reference front() {
        if (empty()) {
            printk("Cannot access front() of empty sequence\n");
            // Still problematic but at least logs the error
            return sequence->_buffer[0];
        }
        return sequence->_buffer[0];
    }
    const_reference front() const {
        if (empty()) {
            printk("Cannot access front() of empty sequence\n");
            // Still problematic but at least logs the error
            return sequence->_buffer[0];
        }
        return sequence->_buffer[0];
    }
    reference back() {
        if (empty()) {
            printk("Cannot access back() of empty sequence\n");
            // Still problematic but at least logs the error
            return sequence->_buffer[0];
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    const_reference back() const {
        if (empty()) {
            printk("Cannot access back() of empty sequence\n");
            // Still problematic but at least logs the error
            return sequence->_buffer[0];
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    
    // Modification
    bool reserve(size_type new_cap) { return ensure_capacity(new_cap); }
    
    bool push_back(const value_type& value) {
        if (owned_sequence) {
            owned_sequence->push_back(value);
            sequence = owned_sequence->get(); // Update pointer in case of reallocation
            return true;
        } else {
            if (!ensure_capacity(sequence->_length + 1)) {
                return false;
            }
            sequence->_buffer[sequence->_length++] = value;
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
            if (new_size > sequence->_maximum && !ensure_capacity(new_size)) {
                return false;
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
    
    bool is_valid() const noexcept { return sequence != nullptr && sequence->_buffer != nullptr; }
};

// Basic helper
template<typename T>
inline Sequence<T> wrap_sequence(T& seq) {
    return Sequence<T>(seq);
}

template<typename T>
inline Sequence<T>& wrap_sequence(Sequence<T>& seq) {
    return seq;  // Already wrapped, return as-is
}

template<typename T>
inline const Sequence<T>& wrap_sequence(const Sequence<T>& seq) {
    return seq;  // Already wrapped, return const ref as-is
}

// Add logging function for debugging
template<typename T>
void log_sequence(const Sequence<T>& seq, const char* name = "Sequence") {
    printk("%s[size=%zu, capacity=%zu]\n", name, seq.size(), seq.capacity());
    
    if (!seq.empty()) {
        printk("Contents:\n");
        for (size_t i = 0; i < (seq.size() < 5 ? seq.size() : 5); ++i) {
            if constexpr (std::is_arithmetic_v<typename Sequence<T>::value_type>) {
                printk("  [%zu]: %d\n", i, static_cast<int>(seq[i]));
            } else {
                printk("  [%zu]: (complex element)\n", i);
            }
        }
        
        if (seq.size() > 5) {
            printk("  ... and %zu more elements\n", seq.size() - 5);
        }
    } else {
        printk("  (empty)\n");
    }
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
