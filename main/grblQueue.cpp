#include "grblQueue.hpp"
#include "esp_log.h"

static const char *TAG = "grblQueue";

// Вспомогательный RAII класс для безопасного взятия/освобождения SemaphoreHandle_t
class MutexLock 
{
private:
    SemaphoreHandle_t _sem;

public:
    explicit MutexLock(SemaphoreHandle_t sem) : _sem(sem) 
    {
        if (_sem) { xSemaphoreTake(_sem, portMAX_DELAY); }
    }

    ~MutexLock() 
    {
        if (_sem) { xSemaphoreGive(_sem); }
    }
};


SafeGrblQueue::SafeGrblQueue() 
{
    _mutex = xSemaphoreCreateMutex();

    if (_mutex == nullptr) 
    {
        ESP_LOGE(TAG, "Failed to create queue mutex");
    }

    _buffer.reset();
}

SafeGrblQueue::~SafeGrblQueue() 
{
    if (_mutex) 
    {
        vSemaphoreDelete(_mutex);
    }
}

bool SafeGrblQueue::push(const grbl_command_t& cmd) 
{
    if (!_mutex) return false;
    MutexLock lock(_mutex);
    
    if (_buffer.is_full()) 
    {
        ESP_LOGW(TAG, "Queue is full");
        return false;
    }

    _buffer.push(cmd);
    return true;
}

bool SafeGrblQueue::push(grbl_command_t&& cmd) 
{
    if (!_mutex) return false;
    MutexLock lock(_mutex);
    
    if (_buffer.is_full()) 
    {
        ESP_LOGW(TAG, "Queue is full");
        return false;
    }

    _buffer.push(std::move(cmd));
    return true;
}

std::optional<grbl_command_t> SafeGrblQueue::pop() 
{
    if (!_mutex) return std::nullopt;
    MutexLock lock(_mutex);
    
    if (_buffer.is_empty()) return std::nullopt;
    
    std::optional<grbl_command_t> item = std::move(*_buffer.curr_data());

    _buffer.pop();

    return item;
}

std::optional<grbl_command_t> SafeGrblQueue::peek() 
{
    if (!_mutex) return std::nullopt;

    MutexLock lock(_mutex);
    
    if (_buffer.is_empty()) return std::nullopt;

    return *_buffer.curr_data();
}

void SafeGrblQueue::clear() 
{
    if (!_mutex) return;

    MutexLock lock(_mutex);

    _buffer.reset_to_zero();
}

bool GrblCommandBuffer::push(const grbl_command_t& cmd, bool high_priority) 
{
    if (high_priority) 
    {
        return _udp_queue.push(cmd);
    }

    return _uart_queue.push(cmd);
}

bool GrblCommandBuffer::push(grbl_command_t&& cmd, bool high_priority) 
{
    if (high_priority) 
        return _udp_queue.push(std::move(cmd));

    return _uart_queue.push(std::move(cmd));
}

std::optional<grbl_command_t> GrblCommandBuffer::peek() 
{
    // Сначала проверяем приоритетную UDP очередь
    if (auto cmd = _udp_queue.peek()) 
    {
        return cmd;
    }

    // Если UDP пуста, смотрим в UART
    return _uart_queue.peek();
}

std::optional<grbl_command_t> GrblCommandBuffer::pop() 
{
    // Сначала выгребаем приоритетные команды из UDP
    if (auto cmd = _udp_queue.pop()) 
    {
        return cmd;
    }

    // Если приоритетных нет, обрабатываем UART
    return _uart_queue.pop();
}

void GrblCommandBuffer::clear_all() 
{
    _udp_queue.clear();
    _uart_queue.clear();
}

GrblCommandBuffer grbl_cmd_dispatcher;
