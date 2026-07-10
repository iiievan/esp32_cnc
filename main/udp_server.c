/*
 * UDP GRBL-like Server for ESP32-S3-ETH (W5500)
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

#define INVALID_SOCKET      -1
#define UDP_BUFFER_SIZE     256

static const char *TAG = "udp_grbl_server";
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

// Функция для генерации локального MAC-адреса
static void get_local_mac(uint8_t *mac_out) 
{
    uint8_t base_mac[6];
    esp_efuse_mac_get_default(base_mac);
    base_mac[0] |= 0x02; 
    memcpy(mac_out, base_mac, 6);
}

void app_main(void)
{
    // 1. Инициализация GPIO ISR
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // 2. Создание event loop и семафора
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    got_ip_sem = xSemaphoreCreateBinary();
    if (got_ip_sem == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    // 3. Инициализация SPI шины
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

    // 4. Конфигурация SPI устройства
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = 25 * 1000 * 1000,
        .spics_io_num = PIN_SPI_CS,
        .queue_size = 20,
    };

    // 5. Инициализация W5500
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &spi_devcfg);
    w5500_config.base.int_gpio_num = PIN_W5500_INT;
    
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 0;
    phy_config.reset_gpio_num = -1;

    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    // 6. Установка драйвера Ethernet
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    // 7. Установка MAC-адреса
    uint8_t local_mac[6];
    get_local_mac(local_mac);
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, local_mac));

    // 8. Инициализация сетевого интерфейса
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // 9. Регистрация обработчиков и запуск
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    // Ждем получения IP
    ESP_LOGI(TAG, "Waiting for IP address...");
    if (xSemaphoreTake(got_ip_sem, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get IP address");
        return;
    }

    // ============= UDP СЕРВЕР =============
    int server_fd = socket(AF_INET, SOCK_DGRAM, 0); // UDP сокет
    if (server_fd < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // Слушать на всех интерфейсах
    server_addr.sin_port = htons(8080);       // Порт 8080

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP socket: errno %d", errno);
        close(server_fd);
        return;
    }

    ESP_LOGI(TAG, "UDP Server listening on port 8080");

    // Буфер для приема данных
    char rx_buffer[UDP_BUFFER_SIZE];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    while (1) {
        // Ожидаем команду от клиента (ПК)
        int len = recvfrom(server_fd, rx_buffer, UDP_BUFFER_SIZE - 1, 0, 
                           (struct sockaddr *)&client_addr, &client_addr_len);
        
        if (len < 0) {
            ESP_LOGE(TAG, "Error receiving UDP data: errno %d", errno);
            continue;
        }

        rx_buffer[len] = '\0'; // Завершаем строку
        ESP_LOGI(TAG, "Received %d bytes from %s:%d: %s", len, 
                 inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), rx_buffer);

        // --- ЛОГИКА GRBL ---
        // Здесь вы можете разбирать команду, например:
        // if (strstr(rx_buffer, "G1") != NULL) { ... двигаем оси ... }
        
        // --- ОТПРАВКА ПОДТВЕРЖДЕНИЯ (ack) ---
        const char *response = "ok\n";
        int sent = sendto(server_fd, response, strlen(response), 0, 
                          (struct sockaddr *)&client_addr, client_addr_len);
        if (sent < 0) {
            ESP_LOGE(TAG, "Error sending response: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "Sent 'ok' to client");
        }
    }

    close(server_fd); // Никогда не достигнется, но для порядка
}