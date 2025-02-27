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
class Sequence {
private:
    static_assert(std::is_member_object_pointer_v<decltype(&T::_length)>, "Type must have _length member");
    static_assert(std::is_member_object_pointer_v<decltype(&T::_buffer)>, "Type must have _buffer member");
    static_assert(std::is_member_object_pointer_v<decltype(&T::_maximum)>, "Type must have _maximum member");
    static_assert(std::is_pointer_v<decltype(T::_buffer)>, "_buffer member must be a pointer type");
    
    T* sequence;
    bool owns_sequence;
    
    // Check and potentially grow the buffer
    bool ensure_capacity(size_t required_capacity) {
        if (required_capacity <= sequence->_maximum) {
            return true;  // Already have enough capacity
        }
        
        // Need to allocate more memory - use fixed growth to avoid excessive allocations
        const size_t max_allowed_capacity = 1024;  // Adjust based on your system constraints
        if (required_capacity > max_allowed_capacity) {
            printk("Requested capacity %zu exceeds maximum allowed (%zu)\n", required_capacity, max_allowed_capacity);
            return false;
        }
        
        size_t new_capacity = std::max(required_capacity, sequence->_maximum * 2);
        if (new_capacity == 0) new_capacity = 16;  // Initial capacity
        
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
    explicit Sequence(T& seq) : sequence(&seq), owns_sequence(false) {
        assert(sequence != nullptr && "Sequence pointer cannot be null");
    }
    
    // Owning constructor with initial capacity
    explicit Sequence(size_type initial_capacity=16) : sequence(static_cast<T*>(k_malloc(sizeof(T)))), owns_sequence(true) {
        if (!sequence) {
            printk("Failed to allocate memory for sequence struct\n");
            return;
        }
        sequence->_buffer = nullptr;
        sequence->_length = 0;
        sequence->_maximum = 0;
        ensure_capacity(initial_capacity);
    }
    
    // Destructor
    ~Sequence() {
        if (owns_sequence && sequence) {
            if (sequence->_buffer) {
                k_free(sequence->_buffer);
            }
            k_free(sequence);
        }
    }
    
    // Prevent copying
    Sequence(const Sequence&) = delete;
    Sequence& operator=(const Sequence&) = delete;
    
    // Allow moving
    Sequence(Sequence&& other) noexcept : sequence(other.sequence), owns_sequence(other.owns_sequence) {
        other.sequence = nullptr;
        other.owns_sequence = false;
    }
    
    Sequence& operator=(Sequence&& other) noexcept {
        if (this != &other) {
            // Clean up our resources
            if (owns_sequence && sequence) {
                if (sequence->_buffer) {
                    k_free(sequence->_buffer);
                }
                k_free(sequence);
            }
            
            sequence = other.sequence;
            owns_sequence = other.owns_sequence;
            
            other.sequence = nullptr;
            other.owns_sequence = false;
        }
        return *this;
    }
    
    T* get_sequence() { return sequence; }
    const T* get_sequence() const { return sequence; }

    // Basic operations
    size_type size() const noexcept { return sequence->_length; }
    bool empty() const noexcept { return sequence->_length == 0; }
    size_type capacity() const noexcept { return sequence->_maximum; }

    // Iterators
    value_type* begin() { return sequence->_buffer; }
    value_type* end() { return sequence->_buffer + sequence->_length; }
    const value_type* begin() const { return sequence->_buffer; }
    const value_type* end() const { return sequence->_buffer + sequence->_length; }

    // Data access
    reference operator[](size_type index) { return sequence->_buffer[index]; }
    const_reference operator[](size_type index) const { return sequence->_buffer[index]; }

    value_type* data() noexcept { return sequence->_buffer; }
    const value_type* data() const noexcept { return sequence->_buffer; }

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
        if (!ensure_capacity(sequence->_length + 1)) {
            return false;
        }
        sequence->_buffer[sequence->_length++] = value;
        return true;
    }
    
    bool pop_back() {
        if (sequence->_length > 0) {
            sequence->_length--;
            return true;
        }
        return false;
    }
    
    bool resize(size_type new_size) {
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

    // Utility    
    void clear() { sequence->_length = 0; }
    bool is_valid() const noexcept { return sequence != nullptr && sequence->_buffer != nullptr; }

    void dump(const char* name = "Sequence") const {
        assert(is_valid() && "Sequence is not valid");
        printk("%s[size=%zu, capacity=%zu]\n", name, size(), capacity());
        
        if (!empty()) {
            printk("Contents:\n");
            for (size_t i = 0; i < (size() < 20 ? size() : 20); ++i) {
                if constexpr (std::is_arithmetic_v<value_type>) {
                    printk("  [%zu]: %d\n", i, static_cast<int>((*this)[i]));
                } else {
                    printk("  [%zu]: (complex element)\n", i);
                }
            }
            
            if (size() > 20) {
                printk("  ... and %zu more elements\n", size() - 20);
            }
        } else {
            printk("  (empty)\n");
        }
    }
};

#endif  // MESSAGE_ADAPTER_HPP_
