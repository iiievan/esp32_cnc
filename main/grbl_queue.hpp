#ifndef GRBL_QUEUE_H
#define GRBL_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <optional>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ring_buffer.hpp" 

#define QUEUE_SIZE (32)
#define MAX_CMD_LENGTH (64)

typedef struct 
{
    char command[MAX_CMD_LENGTH];  
    float target_x;                
    float target_y;                
    float speed;                   
    uint8_t is_motion;             
    uint8_t is_urgent;             
    uint32_t timestamp;            
} grbl_command_t;


/**
 * @brief A wrapper around `ring_buffer` that provides thread safety at the FreeRTOS mutex level
 */
class SafeGrblQueue 
{
private:
    UTILS::ring_buffer<grbl_command_t, QUEUE_SIZE> _buffer;
    SemaphoreHandle_t _mutex;

public:
    SafeGrblQueue();
    ~SafeGrblQueue();

    SafeGrblQueue(const SafeGrblQueue&) = delete;
    SafeGrblQueue& operator=(const SafeGrblQueue&) = delete;

    bool push(const grbl_command_t& cmd);
    bool push(grbl_command_t&& cmd);
    std::optional<grbl_command_t> pop();
    std::optional<grbl_command_t> peek();
    void clear();
    
    inline bool is_empty() const { return _buffer.is_empty(); }
    inline bool is_full() const { return _buffer.is_full(); }
    inline size_t count() const { return _buffer.get_avail(); }
};

/**
 * @brief GRBL Queue Manager (Manages UDP and UART streams)
 */
class GrblCommandBuffer 
{
private:
    SafeGrblQueue _udp_queue;  // high-priority queue 
    SafeGrblQueue _uart_queue; // normal priority queue

public:
    GrblCommandBuffer() = default;

    bool push(const grbl_command_t& cmd, bool high_priority);
    bool push(grbl_command_t&& cmd, bool high_priority);
    std::optional<grbl_command_t> pop();
    std::optional<grbl_command_t> peek();
    void clear_all();
};

extern GrblCommandBuffer grbl_cmd_dispatcher;

#endif // GRBL_QUEUE_H