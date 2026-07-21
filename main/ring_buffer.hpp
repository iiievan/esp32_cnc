#pragma once

#include <cstddef>
#include <cstring>
#include <optional>
#include <utility>
#include <type_traits>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Abstraction for interrupt handling (for portability between Ubuntu, ESP32, and STM32)
// Configure these macros for your target platform
#ifndef LOCK_INTERRUPTS
    #define LOCK_INTERRUPTS()   // For Example: __disable_interrupt(); или taskENTER_CRITICAL();
    #define UNLOCK_INTERRUPTS() // For Example: __enable_interrupt(); или taskEXIT_CRITICAL();
#endif

namespace UTILS {

template <typename T, std::size_t buf_size = 16>
class ring_buffer 
{
private:
    T _queue[buf_size];
    const std::size_t _size;
    std::size_t _avail;
    std::size_t _head;
    std::size_t _tail;

public:
    ring_buffer() : _size(buf_size) 
    { 
        reset(); 
    }

    // The destructor will automatically clear the buffer if T is a complex class
    ~ring_buffer() 
    {
        if constexpr (!std::is_trivially_destructible_v<T>) 
        {
            while (!is_empty()) { pop(); }
        }
    }

    /**
     * @brief Retrieves a single element from the buffer.
     * @return std::nullopt if the buffer is empty.
     */
    std::optional<T> get_one() 
    {
        LOCK_INTERRUPTS();
        std::optional<T> result;
        if (!is_empty()) {
            result = std::move(_queue[_tail]);
            pop();
        }
        UNLOCK_INTERRUPTS();
        return result;
    }

    /**
     * @brief Reads the current element from the output without deleting it.
     */
    std::optional<T> peek_one() 
    {
        LOCK_INTERRUPTS();
        std::optional<T> result;
        if (!is_empty()) 
        {
            result = _queue[_tail];
        }
        UNLOCK_INTERRUPTS();
        return result;
    }

    /**
     * @brief Adds one element to the buffer.
     * If the buffer is full, it overwrites the old element and returns it.
     */
    std::optional<T> add_one(const T& item) 
    {
        LOCK_INTERRUPTS();
        std::optional<T> displaced_item;
        
        if (is_full()) 
        {
            displaced_item = std::move(_queue[_tail]);
            pop(); 
        }

        push(item);
        UNLOCK_INTERRUPTS();
        return displaced_item;
    }

    // Support for Move semantics for large structures (to avoid copying them in memory)
    std::optional<T> add_one(T&& item) 
    {
        LOCK_INTERRUPTS();
        std::optional<T> displaced_item;
        
        if (is_full()) 
        {
            displaced_item = std::move(_queue[_tail]);
            pop();
        }

        push(std::move(item));
        UNLOCK_INTERRUPTS();
        return displaced_item;
    }

    /**
     * @brief Bulk Insertion (for arrays/data buffers)
     */
    std::size_t add(const T* data, std::size_t len) 
    {
        LOCK_INTERRUPTS();
        std::size_t i;
        for (i = 0; i < len; i++) 
        {
            if (is_full()) { break; }
            push(data[i]);
        }
        UNLOCK_INTERRUPTS();
        return i;
    }

    /**
     * @brief Bulk Extraction of Elements
     */
    std::size_t get(T* data, std::size_t len) 
    {
        LOCK_INTERRUPTS();
        std::size_t bytes_avail = get_avail();
        if (len > bytes_avail) { len = bytes_avail; }

        for (std::size_t idx = 0; idx < len; idx++) 
        {
            data[idx] = std::move(_queue[_tail]);
            pop();
        }
        UNLOCK_INTERRUPTS();
        return len;
    }

    inline std::size_t get_avail() const { return _avail; }
    inline std::size_t get_free()  const { return (_size - _avail); }
    inline std::size_t get_size()  const { return _size; }
    inline bool        is_empty()  const { return _avail == 0; }
    inline bool        is_full()   const { return _avail == _size; }
    
    inline void reset() { _head = _tail = _avail = 0; }

    /**
     * @brief Safe zeroing of the buffer based on the data type
     */
    inline void reset_to_zero() 
    {
        _head = _tail = _avail = 0;
        if constexpr (std::is_trivially_copyable_v<T>) 
        {
            // For simple types (char, int, float, POD structures), we use the fast memset
            std::memset(_queue, 0, sizeof(_queue));
        } 
        else 
        {
            // For complex classes, we call destructors/recreate objects by default
            for (auto& item : _queue) { item = T(); }
        }
    }

    inline void set_head(std::size_t value) 
    { 
        _head = value; 
        // When manually setting the head from DMA, we correctly recalculate _avail
        if (_head >= _tail) 
            _avail = _head - _tail;
        else 
            _avail = _size - (_tail - _head);
    }

    inline const T* curr_data() const { return &(_queue[_tail]); }
    inline       T* curr_data()       { return &(_queue[_tail]); }

    inline void pop() 
    {
        if (_avail == 0) return;
        _tail = (_tail + 1) % buf_size;
        _avail--;
    }

    inline void push(const T& data) 
    {
        _queue[_head] = data;
        _head = (_head + 1) % buf_size;
        if (_avail < _size) 
        {
            _avail++;
        } 
        else 
        {
            _tail = (_tail + 1) % buf_size; // Tail shift on overflow
        }
    }

    inline void push(T&& data) 
    {
        _queue[_head] = std::move(data);
        _head = (_head + 1) % buf_size;
        if (_avail < _size) 
        {
            _avail++;
        } 
        else 
        {
            _tail = (_tail + 1) % buf_size;
        }
    }

    inline bool push_for_dma() 
    {
        _head = (_head + 1) % buf_size;
        if (is_full()) 
        {
            _tail = (_tail + 1) % buf_size;
            return false;
        } 
        else 
        {
            _avail++;
            return true;
        }
    }

    T* get_buf_pointer() { return _queue; }
};

} // namespace UTILS