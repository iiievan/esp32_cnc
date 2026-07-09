/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_eth_phy_w5500.h"
#include "esp_eth_mac_w5500.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "errno.h"
//#include "nvs_flash.h" 

#define INVALID_SOCKET      -1
#define SOCKET_MAX_LENGTH   1440 // at least equal to MSS
#define MAX_MSG_LENGTH      128

static const char *TAG = "tcp_client";
static SemaphoreHandle_t got_ip_sem;

// Waveshare ESP32-S3-ETH pins to W5500
#define PIN_SPI_MOSI        (11)
#define PIN_SPI_MISO        (12)
#define PIN_SPI_SCLK        (13)
#define PIN_SPI_CS          (14)
#define PIN_W5500_INT       (10)
#define PIN_W5500_RST       (9)

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    xSemaphoreGive(got_ip_sem);
}

// Функция для генерации локального MAC-адреса на основе базового MAC ESP32
static void get_local_mac(uint8_t *mac_out) 
{
    uint8_t base_mac[6];
    esp_efuse_mac_get_default(base_mac);
    // Генерируем локально-администрируемый MAC (второй бит первого байта = 1)
    base_mac[0] |= 0x02; 
    // Копируем в выходной буфер
    memcpy(mac_out, base_mac, 6);
}

void app_main(void)
{
    //// 1. Инициализация NVS (Необходима для работы DHCP и сетевого стека)
    //esp_err_t ret = nvs_flash_init();
    //if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //    ESP_ERROR_CHECK(nvs_flash_erase());
    //    ret = nvs_flash_init();
    //}
    //ESP_ERROR_CHECK(ret);

    // 2. Инициализация GPIO ISR (для обработки прерываний от W5500)
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // 3. Создание event loop и семафора
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    got_ip_sem = xSemaphoreCreateBinary();
    if (got_ip_sem == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    // 4. Инициализация SPI шины (БЕЗ флага HALFDUPLEX)
    spi_bus_config_t buscfg = 
    {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 5. Конфигурация SPI устройства (полностью как в примере Espressif)
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = 25 * 1000 * 1000, // 25 МГц
        .spics_io_num = PIN_SPI_CS,
        .queue_size = 20,
    };

    // 6. Конфигурация MAC (W5500) - используем макрос ETH_W5500_DEFAULT_CONFIG
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &spi_devcfg);
    w5500_config.base.int_gpio_num = PIN_W5500_INT; // Задаем пин прерывания
    
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);

    // 7. Конфигурация PHY (встроен в W5500)
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 0;
    phy_config.reset_gpio_num = -1; // Сброс через MAC, не нужен отдельный пин

    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    // 8. Установка драйвера Ethernet
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

        uint8_t local_mac[6];
    get_local_mac(local_mac);
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, local_mac));

    // 9. Инициализация сетевого интерфейса и привязка
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // 10. Регистрация обработчиков событий и запуск
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    int transmission_cnt = 0;
    int client_fd = INVALID_SOCKET;

    // Initialize server address
    char txbuffer[MAX_MSG_LENGTH] = {0};
    char rxbuffer[SOCKET_MAX_LENGTH] = {0};
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    if (inet_pton(AF_INET, CONFIG_EXAMPLE_SERVER_IP_ADDRESS, &serv_addr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid address or address not supported: errno %d", errno);
        goto err;
    }
    serv_addr.sin_port = htons(CONFIG_EXAMPLE_SERVER_PORT);

    // Wait until IP address is assigned to this device
    ESP_LOGI(TAG, "Waiting for IP address...");
    if (xSemaphoreTake(got_ip_sem, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get IP address");
        goto err;
    }

    // Main connection loop
    while (1) {
        ESP_LOGI(TAG, "Trying to connect to server...");
        // Create socket if needed
        if (client_fd < 0) {
            client_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (client_fd < 0) {
                ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
                goto err;
            }
        }

        // Try to connect to server
        ESP_LOGI(TAG, "Connecting to server %s:%d", CONFIG_EXAMPLE_SERVER_IP_ADDRESS, CONFIG_EXAMPLE_SERVER_PORT);
        if (connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            ESP_LOGE(TAG, "Failed to connect to server: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "Connected to server");

            // Communication loop - runs until disconnection
            while (1) {
                snprintf(txbuffer, MAX_MSG_LENGTH, "Transmission #%d. Hello from ESP32 TCP client!\n", ++transmission_cnt);
                int bytesSent = send(client_fd, txbuffer, strlen(txbuffer), 0);

                if (bytesSent < 0) {
                    ESP_LOGE(TAG, "Failed to send data: errno %d", errno);
                    break; // Exit inner loop to reconnect
                }

                ESP_LOGI(TAG, "Sent transmission #%d, %d bytes", transmission_cnt, bytesSent);

                // Receive response from server
                int bytesRead = recv(client_fd, rxbuffer, SOCKET_MAX_LENGTH, 0);
                if (bytesRead < 0) {
                    ESP_LOGE(TAG, "Error reading from socket: errno %d", errno);
                    break; // Exit inner loop to reconnect
                } else if (bytesRead == 0) {
                    ESP_LOGW(TAG, "Server closed connection");
                    break; // Exit inner loop to reconnect
                } else {
                    rxbuffer[bytesRead] = '\0'; // Null-terminate the received data
                    ESP_LOGI(TAG, "Received %d bytes: %s", bytesRead, rxbuffer);
                }
                // Delay between transmissions
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        // If we get here, connection was lost, close socket and wait before reconnecting
        if (client_fd != INVALID_SOCKET) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(client_fd, 0);
            close(client_fd);
            client_fd = INVALID_SOCKET;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
err:
    if (got_ip_sem) {
        vSemaphoreDelete(got_ip_sem);
    }
    if (client_fd != INVALID_SOCKET) {
        shutdown(client_fd, 0);
        close(client_fd);
    }
}