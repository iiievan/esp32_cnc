#include "app_tasks.h"
#include "motion/motion.h"
#include <stdio.h>
#include <string.h>
#include "grbl_queue.hpp"
#include "grbl_parser.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "errno.h"
#include "driver/uart.h"
#include "sdkconfig.h"

static const char *TAG_UDP = "udp_grbl_server";
static const char *TAG_UART = "uart_grbl_server";

static bool build_grbl_command(const char *line, grbl_command_t *cmd, parsed_command_t *parsed)
{
    strncpy(cmd->command, line, MAX_CMD_LENGTH - 1);
    cmd->command[MAX_CMD_LENGTH - 1] = '\0';
    cmd->is_motion = 0;
    cmd->is_urgent = 0;

    if (!grbl_parse(line, parsed))
        return false;

    if (parsed->type == CMD_MOTION_LINEAR)
    {
        cmd->is_motion = 1;
        cmd->target_x = parsed->has_x ? parsed->x : 0;
        cmd->target_y = parsed->has_y ? parsed->y : 0;
        cmd->speed = parsed->has_f ? parsed->f : 100;
    }
    else if (parsed->type == CMD_STOP)
    {
        cmd->is_urgent = 1;
    }

    return true;
}

static bool queue_grbl_line(const char *line, bool high_priority, parsed_command_t *parsed)
{
    grbl_command_t cmd;

    if (!build_grbl_command(line, &cmd, parsed)) 
        return false;

    return grbl_cmd_dispatcher.push(std::move(cmd), high_priority);
}

void udp_server_task(void *arg)
{
    int server_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_fd < 0)
    {
        ESP_LOGE(TAG_UDP, "Failed to create UDP socket: errno %d", errno);
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(CONFIG_EXAMPLE_SERVER_PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        ESP_LOGE(TAG_UDP, "Failed to bind UDP socket: errno %d", errno);
        close(server_fd);
        return;
    }

    ESP_LOGI(TAG_UDP, "UDP Server listening on port %d", CONFIG_EXAMPLE_SERVER_PORT);

    char rx_buffer[MAX_CMD_LENGTH];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    while (1)
    {
        int len = recvfrom(server_fd, rx_buffer, MAX_CMD_LENGTH - 1, 0,
                           (struct sockaddr *)&client_addr, &client_addr_len);

        if (len < 0)
        {
            ESP_LOGE(TAG_UDP, "Error receiving UDP data: errno %d", errno);
            continue;
        }

        rx_buffer[len] = '\0';
        ESP_LOGI(TAG_UDP, "Received %d bytes from %s:%d: %s", len,
                 inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), rx_buffer);

        parsed_command_t parsed;
        if (queue_grbl_line(rx_buffer, true, &parsed))
        {
            const char *response = "ok\n";
            sendto(server_fd, response, strlen(response), 0, (struct sockaddr *)&client_addr, client_addr_len);
            ESP_LOGI(TAG_UDP, "CMD queued: %s", grbl_cmd_type_to_string(parsed.type));
        }
        else
        {
            const char *response = "error: Buffer full\n";
            sendto(server_fd, response, strlen(response), 0, (struct sockaddr *)&client_addr, client_addr_len);
            ESP_LOGI(TAG_UDP, "Queue full, sent error");
        }
    }

    close(server_fd);
}

void uart_grbl_task(void *arg)
{
    const uart_port_t uart_num = UART_NUM_1;

    uart_config_t uart_config = {};
    uart_config.baud_rate = CONFIG_GRBL_UART_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num,
                                 CONFIG_GRBL_UART_TX_PIN,
                                 CONFIG_GRBL_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 256 * 2, 256 * 2, 0, NULL, 0));

    ESP_LOGI(TAG_UART, "UART1 GRBL server on TX=%d RX=%d baud=%d",
             CONFIG_GRBL_UART_TX_PIN, CONFIG_GRBL_UART_RX_PIN, CONFIG_GRBL_UART_BAUD_RATE);

    char line_buffer[MAX_CMD_LENGTH];
    size_t line_len = 0;

    while (1)
    {
        uint8_t byte;
        int len = uart_read_bytes(uart_num, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0)
            continue;

        if (byte == '\n' || byte == '\r')
        {
            if (line_len == 0)
                continue;

            line_buffer[line_len] = '\0';
            ESP_LOGI(TAG_UART, "Received: %s", line_buffer);

            parsed_command_t parsed;
            if (queue_grbl_line(line_buffer, false, &parsed))
            {
                const char *response = "ok\n";
                uart_write_bytes(uart_num, response, strlen(response));
                ESP_LOGI(TAG_UART, "CMD queued: %s", grbl_cmd_type_to_string(parsed.type));
            }
            else
            {
                const char *response = "error: Buffer full\n";
                uart_write_bytes(uart_num, response, strlen(response));
                ESP_LOGI(TAG_UART, "Queue full, sent error");
            }

            line_len = 0;
            continue;
        }

        if (line_len < MAX_CMD_LENGTH - 1)
        {
            line_buffer[line_len++] = (char)byte;
        }
        else
        {
            ESP_LOGW(TAG_UART, "Line too long, discarding");
            line_len = 0;
        }
    }
}

void planner_task(void *arg)
{
    while (1)
    {
        if (auto cmd_opt = grbl_cmd_dispatcher.peek())
        {
            grbl_command_t cmd = *cmd_opt;

            if (mr.block_state != BLOCK_RUNNING)
            {  
                if (is_zero_move_default(cmd.target_x, cmd.target_y) || 
                   (!cmd.is_urgent && !cmd.is_motion))
                {
                    grbl_cmd_dispatcher.pop();
                    continue;
                }                    

                if (mp_aline(cmd.target_x, cmd.target_y, cmd.speed))
                {
                    ESP_LOGI(TAG_UDP, "Planner: move started to (%.2f, %.2f, %.2f)",
                             cmd.target_x, cmd.target_y, cmd.speed);
                             
                    grbl_cmd_dispatcher.pop();

                    // Wait for completion ONLY if the movement is actually taking place
                    // (if the position has changed and the timer has started)
                    if (mr.block_state != BLOCK_IDLE || motion_complete_sem != NULL)
                    {
                        xSemaphoreTake(motion_complete_sem, portMAX_DELAY);
                    }
                }
                else
                {
                    ESP_LOGE(TAG_UDP, "Planner: move failed");
                    vTaskDelay(pdMS_TO_TICKS(10)); 
                }
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
