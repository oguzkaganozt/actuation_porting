#ifndef COMMON__SEQUENCE_HPP_
#define COMMON__SEQUENCE_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

#define DEFAULT_SEQUENCE_SIZE 128
#define MAX_SEQUENCE_SIZE 2048

/**
 * @brief Sequence class as a wrapper for DDS sequences
 * @tparam T Type of the sequence
 */
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
            fprintf(stderr, "Invalid sequence state\n");
            std::exit(EXIT_FAILURE);
        }
        
        if (required_capacity <= sequence->_maximum) {
            return true;  // Already have enough capacity
        }
        
        // Need to allocate more memory - use fixed growth to avoid excessive allocations
        if (required_capacity > MAX_SEQUENCE_SIZE) {
            fprintf(stderr, "Requested capacity %zu exceeds maximum allowed (%zu)\n", required_capacity, MAX_SEQUENCE_SIZE);
            std::exit(EXIT_FAILURE);
        }
        
        size_t new_capacity = (sequence->_maximum * 2 > required_capacity) ? 
                               sequence->_maximum * 2 : required_capacity;
        if (new_capacity == 0) new_capacity = DEFAULT_SEQUENCE_SIZE;
        
        // Use typed malloc for better type safety
        using ElementType = typename std::remove_pointer<decltype(T::_buffer)>::type;
        
        // Size overflow check
        if (new_capacity > SIZE_MAX / sizeof(ElementType)) {
            fprintf(stderr, "Allocation size would overflow\n");
            std::exit(EXIT_FAILURE);
        }
        
        ElementType* new_buffer = static_cast<ElementType*>(malloc(new_capacity * sizeof(ElementType)));
        if (!new_buffer) {
            fprintf(stderr, "Failed to allocate memory for sequence buffer\n");
            std::exit(EXIT_FAILURE);
        }
        
        // Copy existing elements
        if (sequence->_buffer && sequence->_length > 0) {
            std::memcpy(new_buffer, sequence->_buffer, sequence->_length * sizeof(ElementType));
        }
        
        // Free old buffer
        if (sequence->_buffer) {
            free(sequence->_buffer);
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
    explicit Sequence(size_type initial_capacity=DEFAULT_SEQUENCE_SIZE) : sequence(static_cast<T*>(malloc(sizeof(T)))), owns_sequence(true) {
        if (!sequence) {
            fprintf(stderr, "Failed to allocate memory for sequence struct\n");
            std::exit(EXIT_FAILURE);
        }
        sequence->_buffer = nullptr;
        sequence->_length = 0;
        sequence->_maximum = 0;
        if (!ensure_capacity(initial_capacity)) {
            // If buffer allocation fails, clean up and flag as non-owning
            free(sequence);
            sequence = nullptr;
            owns_sequence = false;
        }
    }
    
    // Destructor //TODO: Check if this is correct
    ~Sequence() {
        if (owns_sequence && sequence) {
            if (sequence->_buffer) {
                free(const_cast<void*>(static_cast<const void*>(sequence->_buffer)));
            }
            free(const_cast<void*>(static_cast<const void*>(sequence)));
        }
    }
    
    // Delete copy operations
    Sequence(const Sequence& other) = delete;
    Sequence& operator=(const Sequence& other) = delete;

    // Add move operations
    Sequence(Sequence&& other) noexcept : sequence(other.sequence), owns_sequence(other.owns_sequence) {
        // Transfer ownership
        other.sequence = nullptr;
        other.owns_sequence = false;
    }

    Sequence& operator=(Sequence&& other) noexcept {
        if (this != &other) {
            // Clean up our resources first
            if (owns_sequence && sequence) {
                if (sequence->_buffer) {
                    free(sequence->_buffer);
                }
                free(sequence);
            }
            
            // Transfer ownership
            sequence = other.sequence;
            owns_sequence = other.owns_sequence;
            other.sequence = nullptr;
            other.owns_sequence = false;
        }
        return *this;
    }
    
    // Basic operations
    size_type size() const noexcept { return sequence ? sequence->_length : 0; }
    size_type max_size() const noexcept { return MAX_SEQUENCE_SIZE; }
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
            fprintf(stderr, "Dereferencing invalid sequence\n");
            std::exit(EXIT_FAILURE);
        }
        if(index >= sequence->_length) {
            fprintf(stderr, "Index %zu out of bounds (size: %zu)\n", index, sequence->_length);
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[index]; 
    }
    
    const_reference operator[](size_type index) const { 
        if (!sequence || !sequence->_buffer) {
            fprintf(stderr, "Dereferencing invalid sequence\n");
            std::exit(EXIT_FAILURE);
        }
        if(index >= sequence->_length) {
            fprintf(stderr, "Index %zu out of bounds (size: %zu)\n", index, sequence->_length);
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[index]; 
    }

    value_type* data() noexcept { 
        if (!sequence || !sequence->_buffer) {
            fprintf(stderr, "Invalid sequence\n");
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer; 
    }
    
    const value_type* data() const noexcept { 
        if (!sequence || !sequence->_buffer) {
            fprintf(stderr, "Invalid sequence\n");
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer; 
    }

    reference at(size_type index) {
        if (!sequence || !sequence->_buffer || index >= sequence->_length) {
            fprintf(stderr, "Index %zu out of range (size: %zu)\n", index, size());
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[index];
    }

    const_reference at(size_type index) const {
        if (!sequence || !sequence->_buffer || index >= sequence->_length) {
            fprintf(stderr, "Index %zu out of range (size: %zu)\n", index, size());
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[index];
    }
    
    reference front() {
        if (!sequence || !sequence->_buffer || empty()) {
            fprintf(stderr, "Cannot access front() of empty sequence\n");
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[0];
    }
    
    const_reference front() const {
        if (!sequence || !sequence->_buffer || empty()) {
            fprintf(stderr, "Cannot access front() of empty sequence\n");
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[0];
    }
    
    reference back() {
        if (!sequence || !sequence->_buffer || empty()) {
            fprintf(stderr, "Cannot access back() of empty sequence\n");
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    
    const_reference back() const {
        if (!sequence || !sequence->_buffer || empty()) {
            fprintf(stderr, "Cannot access back() of empty sequence\n");
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    
    // Modification
    bool reserve(size_type new_cap) { return ensure_capacity(new_cap); }
    void clear() { if (sequence) sequence->_length = 0; }
    
    bool push_back(const value_type& value) {
        if (!sequence || !ensure_capacity(sequence->_length + 1)) {
            fprintf(stderr, "Failed to push_back() - copy\n");
            std::exit(EXIT_FAILURE);
        }
        sequence->_buffer[sequence->_length++] = value;
        return true;
    }
    
    // Add move-enabled push_back for better performance with movable types
    bool push_back(value_type&& value) {
        if (!sequence || !ensure_capacity(sequence->_length + 1)) {
            fprintf(stderr, "Failed to push_back() - move\n");
            std::exit(EXIT_FAILURE);
        }
        sequence->_buffer[sequence->_length++] = std::move(value);
        return true;
    }
    
    bool pop_back() {
        if (!sequence || sequence->_length == 0) {
            fprintf(stderr, "Cannot pop_back() from empty sequence\n");
            std::exit(EXIT_FAILURE);
        }
        sequence->_length--;
        return true;
    }
    
    bool resize(size_type new_size) {
        if (!sequence) {
            fprintf(stderr, "Invalid sequence\n");
            std::exit(EXIT_FAILURE);
        }
        
        if (new_size > sequence->_maximum && !ensure_capacity(new_size)) {
            fprintf(stderr, "Failed to resize sequence\n");
            std::exit(EXIT_FAILURE);
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
    T* get_sequence() { return sequence; }
    const T* get_sequence() const { return sequence; }
    bool is_valid() const noexcept { return sequence != nullptr && sequence->_buffer != nullptr; }
    
    void dump(const char* name = "Sequence") const {
        if (!is_valid()) {
            fprintf(stderr, "%s: Invalid sequence\n", name);
            return;
        }
        
        fprintf(stderr, "%s[size=%zu, capacity=%zu]\n", name, size(), capacity());
        
        if (!empty()) {
            fprintf(stderr, "Contents:\n");
            for (size_t i = 0; i < (size() < 20 ? size() : 20); ++i) {
                if constexpr (std::is_integral_v<value_type>) {
                    fprintf(stderr, "  [%zu]: %d\n", i, static_cast<int>((*this)[i]));
                } else if constexpr (std::is_floating_point_v<value_type>) {
                    fprintf(stderr, "  [%zu]: %f\n", i, static_cast<double>((*this)[i]));
                } else {
                    fprintf(stderr, "  [%zu]: (unformattable type)\n", i);
                }
            }
            
            if (size() > 20) {
                fprintf(stderr, "  ... and %zu more elements\n", size() - 20);
            }
        } else {
            fprintf(stderr, "  (empty)\n");
        }
    }
};

// Specialization for const types
template<typename T>
class Sequence<const T> {
private:
    const T* sequence;
    bool owns_sequence;

public:
    using value_type = typename std::remove_pointer<decltype(T::_buffer)>::type;
    using reference = const value_type&;
    using const_reference = const value_type&;
    using size_type = size_t;
    
    // Non-owning constructor
    explicit Sequence(const T& seq) : sequence(&seq), owns_sequence(false) {
        assert(sequence != nullptr && "Sequence pointer cannot be null");
    }
    
    // Owning constructor with initial capacity - NOT supported for const types
    explicit Sequence(size_type initial_capacity=DEFAULT_SEQUENCE_SIZE) : sequence(nullptr), owns_sequence(false) {
        fprintf(stderr, "Cannot create owning Sequence with const type\n");
        std::exit(EXIT_FAILURE);
    }
    
    // Destructor - only delete non-const data
    ~Sequence() {
        // No memory management for const types
    }
    
    // Delete copy operations
    Sequence(const Sequence& other) = delete;
    Sequence& operator=(const Sequence& other) = delete;

    // Add move operations
    Sequence(Sequence&& other) noexcept : sequence(other.sequence), owns_sequence(other.owns_sequence) {
        // Transfer ownership
        other.sequence = nullptr;
        other.owns_sequence = false;
    }

    Sequence& operator=(Sequence&& other) noexcept {
        if (this != &other) {
            // Transfer ownership
            sequence = other.sequence;
            owns_sequence = other.owns_sequence;
            other.sequence = nullptr;
            other.owns_sequence = false;
        }
        return *this;
    }
    
    // Read-only access methods
    size_type size() const noexcept { return sequence ? sequence->_length : 0; }
    size_type max_size() const noexcept { return MAX_SEQUENCE_SIZE; }
    bool empty() const noexcept { return !sequence || sequence->_length == 0; }
    size_type capacity() const noexcept { return sequence ? sequence->_maximum : 0; }

    // Iterators - const only
    const value_type* begin() const { return sequence ? sequence->_buffer : nullptr; }
    const value_type* end() const { return sequence ? sequence->_buffer + sequence->_length : nullptr; }

    // Data access - const only
    const_reference operator[](size_type index) const { 
        if (!sequence || !sequence->_buffer) {
            fprintf(stderr, "Dereferencing invalid sequence\n");
            std::exit(EXIT_FAILURE);
        }
        if(index >= sequence->_length) {
            fprintf(stderr, "Index %zu out of bounds (size: %zu)\n", index, sequence->_length);
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[index]; 
    }
    
    const value_type* data() const noexcept { 
        if (!sequence || !sequence->_buffer) {
            fprintf(stderr, "Invalid sequence\n");
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer; 
    }

    const_reference at(size_type index) const {
        if (!sequence || !sequence->_buffer || index >= sequence->_length) {
            fprintf(stderr, "Index %zu out of range (size: %zu)\n", index, size());
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[index];
    }
    
    const_reference front() const {
        if (!sequence || !sequence->_buffer || empty()) {
            fprintf(stderr, "Cannot access front() of empty sequence\n");
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[0];
    }
    
    const_reference back() const {
        if (!sequence || !sequence->_buffer || empty()) {
            fprintf(stderr, "Cannot access back() of empty sequence\n");
            std::exit(EXIT_FAILURE);
        }
        return sequence->_buffer[sequence->_length - 1];
    }
    
    // For const specialization, modification methods return a new sequence with copied data
    Sequence<value_type> copy() const {
        Sequence<value_type> result(size());
        for (size_type i = 0; i < size(); ++i) {
            result.push_back((*this)[i]);
        }
        return result;
    }
    
    // These methods are maintained for API compatibility but create a new non-const sequence
    Sequence<value_type> removeOverlapPoints(size_t start_idx = 0) const {
        Sequence<value_type> result(size());
        
        // Implementation based on original removeOverlapPoints
        for (size_t i = 0; i <= start_idx && i < size(); ++i) {
            result.push_back(at(i));
        }
        
        constexpr double eps = 1.0E-08;
        for (size_t i = start_idx + 1; i < size(); ++i) {
            const auto prev_p = autoware::universe_utils::getPoint(result.back());
            const auto curr_p = autoware::universe_utils::getPoint(at(i));
            if (std::abs(prev_p.x - curr_p.x) < eps && std::abs(prev_p.y - curr_p.y) < eps) {
                continue;
            }
            result.push_back(at(i));
        }
        
        return result;
    }

    // Utility
    const T* get_sequence() const { return sequence; }
    bool is_valid() const noexcept { return sequence != nullptr && sequence->_buffer != nullptr; }
    
    void dump(const char* name = "Sequence") const {
        if (!is_valid()) {
            fprintf(stderr, "%s: Invalid sequence\n", name);
            return;
        }
        
        fprintf(stderr, "%s[size=%zu, capacity=%zu]\n", name, size(), capacity());
        
        if (!empty()) {
            fprintf(stderr, "Contents:\n");
            for (size_t i = 0; i < (size() < 20 ? size() : 20); ++i) {
                if constexpr (std::is_integral_v<value_type>) {
                    fprintf(stderr, "  [%zu]: %d\n", i, static_cast<int>((*this)[i]));
                } else if constexpr (std::is_floating_point_v<value_type>) {
                    fprintf(stderr, "  [%zu]: %f\n", i, static_cast<double>((*this)[i]));
                } else {
                    fprintf(stderr, "  [%zu]: (unformattable type)\n", i);
                }
            }
            
            if (size() > 20) {
                fprintf(stderr, "  ... and %zu more elements\n", size() - 20);
            }
        } else {
            fprintf(stderr, "  (empty)\n");
        }
    }
};

template<typename T>
Sequence<T> wrap(T& seq) { return Sequence<T>(seq); }
template<typename T>
Sequence<T> wrap(T* seq) { return Sequence<T>(seq); }

#endif  // COMMON__SEQUENCE_HPP_