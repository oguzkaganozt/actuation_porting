#ifndef MESSAGE_ADAPTER_HPP_
#define MESSAGE_ADAPTER_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define DEFAULT_SEQUENCE_SIZE 16
#define MAX_SEQUENCE_SIZE 2048

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
        if (!sequence || !sequence->_buffer) {
            printk("Invalid sequence state\n");
            k_panic();
        }
        
        if (required_capacity <= sequence->_maximum) {
            return true;  // Already have enough capacity
        }
        
        // Need to allocate more memory - use fixed growth to avoid excessive allocations
        if (required_capacity > MAX_SEQUENCE_SIZE) {
            printk("Requested capacity %zu exceeds maximum allowed (%zu)\n", required_capacity, MAX_SEQUENCE_SIZE);
            k_panic();
        }
        
        size_t new_capacity = (sequence->_maximum * 2 > required_capacity) ? 
                               sequence->_maximum * 2 : required_capacity;
        if (new_capacity == 0) new_capacity = DEFAULT_SEQUENCE_SIZE;
        
        // Use typed malloc for better type safety
        using ElementType = typename std::remove_pointer<decltype(T::_buffer)>::type;
        
        // Size overflow check
        if (new_capacity > SIZE_MAX / sizeof(ElementType)) {
            printk("Allocation size would overflow\n");
            k_panic();
        }
        
        ElementType* new_buffer = static_cast<ElementType*>(k_malloc(new_capacity * sizeof(ElementType)));
        if (!new_buffer) {
            printk("Failed to allocate memory for sequence buffer\n");
            k_panic();
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
    explicit Sequence(size_type initial_capacity=DEFAULT_SEQUENCE_SIZE) : sequence(static_cast<T*>(k_malloc(sizeof(T)))), owns_sequence(true) {
        if (!sequence) {
            printk("Failed to allocate memory for sequence struct\n");
            k_panic();
        }
        sequence->_buffer = nullptr;
        sequence->_length = 0;
        sequence->_maximum = 0;
        if (!ensure_capacity(initial_capacity)) {
            // If buffer allocation fails, clean up and flag as non-owning
            k_free(sequence);
            sequence = nullptr;
            owns_sequence = false;
        }
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
    
    // Copy constructor
    Sequence(const Sequence& other) : sequence(static_cast<T*>(k_malloc(sizeof(T)))), owns_sequence(true) {
        if (!sequence) {
            printk("Failed to allocate memory for sequence struct in copy constructor\n");
            k_panic();
        }
        
        // Initialize the sequence structure
        sequence->_buffer = nullptr;
        sequence->_length = 0;
        sequence->_maximum = 0;
        
        // Allocate and copy the buffer if the source has one
        if (other.sequence && other.sequence->_buffer && other.sequence->_length > 0) {
            // Allocate with exact capacity to match original
            if (!ensure_capacity(other.sequence->_maximum)) {
                printk("Failed to allocate buffer in copy constructor\n");
                k_free(sequence);
                sequence = nullptr;
                owns_sequence = false;
                k_panic();
            }
            
            // Copy each element
            // TODO: Check for trait support within Zephyr
            if (std::is_trivially_copyable<value_type>::value) {
                std::memcpy(sequence->_buffer, other.sequence->_buffer, 
                           sequence->_length * sizeof(value_type));
            } else {
                for (size_t i = 0; i < sequence->_length; i++) {
                    sequence->_buffer[i] = other.sequence->_buffer[i];
                }
            }
        }
    }
    
    // Copy assignment operator
    Sequence& operator=(const Sequence& other) {
        if (this != &other) {  // Self-assignment check
            // Clean up existing resources
            if (owns_sequence && sequence) {
                if (sequence->_buffer) {
                    k_free(sequence->_buffer);
                    sequence->_buffer = nullptr;
                }
            }
            
            // If we don't own a sequence structure, create one
            if (!sequence) {
                sequence = static_cast<T*>(k_malloc(sizeof(T)));
                if (!sequence) {
                    printk("Failed to allocate memory for sequence struct in copy assignment\n");
                    k_panic();
                }
                owns_sequence = true;
                sequence->_buffer = nullptr;
                sequence->_length = 0;
                sequence->_maximum = 0;
            }
            
            // Allocate and copy the buffer
            if (other.sequence && other.sequence->_buffer && other.sequence->_length > 0) {
                if (!ensure_capacity(other.sequence->_maximum)) {
                    printk("Failed to allocate buffer in copy assignment\n");
                    k_panic();
                }
                
                // Copy each element
                // TODO: Check for trait support within Zephyr
                if (std::is_trivially_copyable<value_type>::value) {
                    std::memcpy(sequence->_buffer, other.sequence->_buffer, 
                               sequence->_length * sizeof(value_type));
                } else {
                    for (size_t i = 0; i < sequence->_length; i++) {
                        sequence->_buffer[i] = other.sequence->_buffer[i];
                    }
                }
            } else {
                // Other sequence is empty
                sequence->_length = 0;
            }
        }
        return *this;
    }
    
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

    // Basic operations
    size_type size() const noexcept { return sequence ? sequence->_length : 0; }
    bool empty() const noexcept { return !sequence || sequence->_length == 0; }
    size_type capacity() const noexcept { return sequence ? sequence->_maximum : 0; }

    // Iterators
    value_type* begin() { return sequence ? sequence->_buffer : nullptr; }
    value_type* end() { return sequence ? sequence->_buffer + sequence->_length : nullptr; }
    const value_type* begin() const { return sequence ? sequence->_buffer : nullptr; }
    const value_type* end() const { return sequence ? sequence->_buffer + sequence->_length : nullptr; }

    // Data access
    reference operator[](size_type index) { 
        if (!sequence || !sequence->_buffer) {
            printk("Dereferencing invalid sequence\n");
            k_panic();
        }
        if(index >= sequence->_length) {
            printk("Index %zu out of bounds (size: %zu)\n", index, sequence->_length);
            k_panic();
        }
        return sequence->_buffer[index]; 
    }
    
    const_reference operator[](size_type index) const { 
        if (!sequence || !sequence->_buffer) {
            printk("Dereferencing invalid sequence\n");
            k_panic();
        }
        if(index >= sequence->_length) {
            printk("Index %zu out of bounds (size: %zu)\n", index, sequence->_length);
            k_panic();
        }
        return sequence->_buffer[index]; 
    }

    value_type* data() noexcept { 
        if (!sequence || !sequence->_buffer) {
            printk("Invalid sequence\n");
            k_panic();
        }
        return sequence->_buffer; 
    }
    const value_type* data() const noexcept { 
        if (!sequence || !sequence->_buffer) {
            printk("Invalid sequence\n");
            k_panic();
        }
        return sequence->_buffer; 
    }

    reference at(size_type index) {
        if (!sequence || !sequence->_buffer || index >= sequence->_length) {
            printk("Index %zu out of range (size: %zu)\n", index, size());
            k_panic();
        }
        return sequence->_buffer[index];
    }

    const_reference at(size_type index) const {
        if (!sequence || !sequence->_buffer || index >= sequence->_length) {
            printk("Index %zu out of range (size: %zu)\n", index, size());
            k_panic();
        }
        return sequence->_buffer[index];
    }
    
    reference front() {
        if (!sequence || !sequence->_buffer || empty()) {
            printk("Cannot access front() of empty sequence\n");
            k_panic();
        }
        return sequence->_buffer[0];
    }
    
    const_reference front() const {
        if (!sequence || !sequence->_buffer || empty()) {
            printk("Cannot access front() of empty sequence\n");
            k_panic();
        }
        return sequence->_buffer[0];
    }
    
    reference back() {
        if (!sequence || !sequence->_buffer || empty()) {
            printk("Cannot access back() of empty sequence\n");
            k_panic();
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    
    const_reference back() const {
        if (!sequence || !sequence->_buffer || empty()) {
            printk("Cannot access back() of empty sequence\n");
            k_panic();
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    
    // Modification
    bool reserve(size_type new_cap) { return ensure_capacity(new_cap); }
    void clear() { if (sequence) sequence->_length = 0; }
    
    bool push_back(const value_type& value) {
        if (!sequence || !ensure_capacity(sequence->_length + 1)) {
            printk("Failed to push_back() - copy\n");
            k_panic();
        }
        sequence->_buffer[sequence->_length++] = value;
        return true;
    }
    
    // Add move-enabled push_back for better performance with movable types
    bool push_back(value_type&& value) {
        if (!sequence || !ensure_capacity(sequence->_length + 1)) {
            printk("Failed to push_back() - move\n");
            k_panic();
        }
        sequence->_buffer[sequence->_length++] = std::move(value);
        return true;
    }
    
    bool pop_back() {
        if (!sequence || sequence->_length == 0) {
            printk("Cannot pop_back() from empty sequence\n");
            k_panic();
        }
        sequence->_length--;
        return true;
    }
    
    bool resize(size_type new_size) {
        if (!sequence) {
            printk("Invalid sequence\n");
            k_panic();
        }
        
        if (new_size > sequence->_maximum && !ensure_capacity(new_size)) {
            printk("Failed to resize sequence\n");
            k_panic();
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
    static Sequence wrap(T& seq) { return Sequence(seq); }
    static Sequence wrap(T* seq) { return Sequence(seq); }
    T* get_sequence() { return sequence; }
    const T* get_sequence() const { return sequence; }
    bool is_valid() const noexcept { return sequence != nullptr && sequence->_buffer != nullptr; }
    
    void dump(const char* name = "Sequence") const {
        if (!is_valid()) {
            printk("%s: Invalid sequence\n", name);
            return;
        }
        
        printk("%s[size=%zu, capacity=%zu]\n", name, size(), capacity());
        
        if (!empty()) {
            printk("Contents:\n");
            for (size_t i = 0; i < (size() < 20 ? size() : 20); ++i) {
                if constexpr (std::is_integral_v<value_type>) {
                    printk("  [%zu]: %d\n", i, static_cast<int>((*this)[i]));
                } else if constexpr (std::is_floating_point_v<value_type>) {
                    printk("  [%zu]: %f\n", i, static_cast<double>((*this)[i]));
                } else {
                    printk("  [%zu]: (unformattable type)\n", i);
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
