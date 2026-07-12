// main/grbl_queue.h
#ifndef GRBL_QUEUE_H
#define GRBL_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QUEUE_SIZE 128
#define MAX_CMD_LENGTH 64

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


typedef struct 
{
    grbl_command_t buffer[QUEUE_SIZE];
    volatile uint8_t head;         
    volatile uint8_t tail;         
    volatile uint8_t count;        
    SemaphoreHandle_t mutex;       
} grbl_queue_t;


void grbl_queue_init(grbl_queue_t *queue);

bool grbl_queue_push(grbl_queue_t *queue, const grbl_command_t *cmd);

bool grbl_queue_pop(grbl_queue_t *queue, grbl_command_t *cmd);

bool grbl_queue_peek(grbl_queue_t *queue, grbl_command_t *cmd);

static inline bool grbl_queue_is_empty(const grbl_queue_t *queue) { return queue->count == 0; }

static inline bool grbl_queue_is_full(const grbl_queue_t *queue) { return queue->count >= QUEUE_SIZE; }

static inline uint8_t grbl_queue_count(const grbl_queue_t *queue) { return queue->count; }

void grbl_queue_clear(grbl_queue_t *queue);

#ifdef __cplusplus
}
#endif

#endif // GRBL_QUEUE_H