#include "grbl_queue.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "grbl_queue";

void grbl_queue_init(grbl_queue_t *queue) 
{
    memset(queue->buffer, 0, sizeof(queue->buffer));
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->mutex = xSemaphoreCreateMutex();
    
    if (queue->mutex == NULL) 
        ESP_LOGE(TAG, "Failed to create queue mutex");

}

bool grbl_queue_push(grbl_queue_t *queue, const grbl_command_t *cmd) 
{
    if (queue->mutex == NULL) 
        return false;
    
    xSemaphoreTake(queue->mutex, portMAX_DELAY);
    
    if (queue->count >= QUEUE_SIZE) 
    {
        xSemaphoreGive(queue->mutex);
        ESP_LOGW(TAG, "Queue is full");
        return false;
    }
    
    memcpy(&queue->buffer[queue->head], cmd, sizeof(grbl_command_t));
    queue->head = (queue->head + 1) % QUEUE_SIZE;
    queue->count++;
    
    xSemaphoreGive(queue->mutex);
    return true;
}

bool grbl_queue_pop(grbl_queue_t *queue, grbl_command_t *cmd) 
{
    if (queue->mutex == NULL) 
        return false;
    
    xSemaphoreTake(queue->mutex, portMAX_DELAY);
    
    if (queue->count == 0) 
    {
        xSemaphoreGive(queue->mutex);
        ESP_LOGW(TAG, "Queue is empty");
        return false;
    }
    
    memcpy(cmd, &queue->buffer[queue->tail], sizeof(grbl_command_t));
    queue->tail = (queue->tail + 1) % QUEUE_SIZE;
    queue->count--;
    
    xSemaphoreGive(queue->mutex);
    return true;
}

bool grbl_queue_peek(grbl_queue_t *queue, grbl_command_t *cmd) 
{
    if (queue->mutex == NULL) 
        return false;
    
    xSemaphoreTake(queue->mutex, portMAX_DELAY);
    
    if (queue->count == 0) 
    {
        xSemaphoreGive(queue->mutex);
        return false;
    }
    
    memcpy(cmd, &queue->buffer[queue->tail], sizeof(grbl_command_t));
    xSemaphoreGive(queue->mutex);
    return true;
}

void grbl_queue_clear(grbl_queue_t *queue) 
{
    if (queue->mutex == NULL) 
        return;
    
    xSemaphoreTake(queue->mutex, portMAX_DELAY);
    
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    memset(queue->buffer, 0, sizeof(queue->buffer));

    xSemaphoreGive(queue->mutex);
}

void grbl_cmd_queues_init(grbl_cmd_queues_t *queues)
{
    grbl_queue_init(&queues->udp);
    grbl_queue_init(&queues->uart);
}

bool grbl_cmd_queues_push(grbl_cmd_queues_t *queues, const grbl_command_t *cmd, bool high_priority)
{
    if (high_priority)
        return grbl_queue_push(&queues->udp, cmd);

    return grbl_queue_push(&queues->uart, cmd);
}

bool grbl_cmd_queues_peek(grbl_cmd_queues_t *queues, grbl_command_t *cmd)
{
    if (grbl_queue_peek(&queues->udp, cmd))
        return true;

    return grbl_queue_peek(&queues->uart, cmd);
}

bool grbl_cmd_queues_pop(grbl_cmd_queues_t *queues, grbl_command_t *cmd)
{
    if (grbl_queue_pop(&queues->udp, cmd))
        return true;

    return grbl_queue_pop(&queues->uart, cmd);
}