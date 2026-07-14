#include "app_tasks.h"
#include "motion/motion.h"
#include <stdio.h>
#include <string.h>
#include "grbl_queue.h"
#include "grbl_parser.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "errno.h"

static const char *TAG = "udp_grbl_server";

static grbl_queue_t cmd_queue;

void udp_server_task(void *arg)
{
    int server_fd = socket(AF_INET, SOCK_DGRAM, 0); // UDP socket
    if (server_fd < 0) 
    {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // Listen on any interfaces
    server_addr.sin_port = htons(8080);       // Port 8080

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) 
    {
        ESP_LOGE(TAG, "Failed to bind UDP socket: errno %d", errno);
        close(server_fd);
        return;
    }

    ESP_LOGI(TAG, "UDP Server listening on port 8080");

    char rx_buffer[UDP_BUFFER_SIZE];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    
    grbl_queue_init(&cmd_queue);

    while (1) 
    {
        // Wait CMD
        int len = recvfrom(server_fd, rx_buffer, UDP_BUFFER_SIZE - 1, 0, 
                           (struct sockaddr *)&client_addr, &client_addr_len);
        
        if (len < 0) {
            ESP_LOGE(TAG, "Error receiving UDP data: errno %d", errno);
            continue;
        }

        rx_buffer[len] = '\0'; 
        ESP_LOGI(TAG, "Received %d bytes from %s:%d: %s", len, 
                 inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), rx_buffer);

        // Parce CMD
        grbl_command_t cmd;
        parsed_command_t parsed;
        strcpy(cmd.command, rx_buffer);
        cmd.is_motion = 0;
        cmd.is_urgent = 0;

        if (grbl_parse(rx_buffer, &parsed)) 
        {
            if (parsed.type == CMD_MOTION_LINEAR ) 
            {
                cmd.is_motion   = 1;
                cmd.target_x    = parsed.has_x ? parsed.x : 0;
                cmd.target_y    = parsed.has_y ? parsed.y : 0;
                cmd.speed       = parsed.has_f ? parsed.f : 100;
            } 
            else if (parsed.type == CMD_STOP) 
            {
                cmd.is_urgent = 1;
            }
        }

        // Push cmd in queue
        if (grbl_queue_push(&cmd_queue, &cmd)) 
        {
            const char *response = "ok\n";
            sendto(server_fd, response, strlen(response), 0,(struct sockaddr *)&client_addr, client_addr_len);
            const char *cmd_type = grbl_cmd_type_to_string(parsed.type);
            ESP_LOGI(TAG, "CMD queued: %s", cmd_type);

        }
        else
        {
            const char *response = "error: Buffer full\n";
            sendto(server_fd, response, strlen(response), 0, (struct sockaddr *)&client_addr, client_addr_len);
            ESP_LOGI(TAG, "Queue full, sent error");
        }        
    }

    close(server_fd); 
}

void planner_task(void *arg)
{
    grbl_command_t cmd;
    
    while (1) 
    {
        // Смотрим, есть ли команда в очереди (peek)
        if (grbl_queue_peek(&cmd_queue, &cmd)) 
        {
            // Проверяем, не занят ли планировщик
            if (mr.block_state != BLOCK_RUNNING) 
            {
                grbl_queue_pop(&cmd_queue, &cmd);
                
                // Выполняем
                if (mp_aline(cmd.target_x, cmd.target_y, cmd.speed)) 
                {
                    ESP_LOGI(TAG, "Planner: move started to (%.2f, %.2f, %.2f)", 
                             cmd.target_x, cmd.target_y, cmd.speed);
                }
                else
                {
                    ESP_LOGE(TAG, "Planner: move failed");
                }
            }
            else
            {
                // Планировщик занят - ждем
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        else
        {
            // Очередь пуста - ждем
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
