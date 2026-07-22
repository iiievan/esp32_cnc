#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_tasks.h"
#include "motion/motion.hpp"
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
#include "motion_planner.hpp"

static const char *TAG = "ethernet_init";
static SemaphoreHandle_t got_ip_sem;
extern MotionPlanner* G_plnr;

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

// Generate local MAC-adress
static void get_local_mac(uint8_t *mac_out) 
{
    uint8_t base_mac[6];
    esp_efuse_mac_get_default(base_mac);
    base_mac[0] |= 0x02; 
    memcpy(mac_out, base_mac, 6);
}

extern "C" void app_main()
{
    // Init GPIO ISR for W5500 SPI
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    got_ip_sem = xSemaphoreCreateBinary();
    if (got_ip_sem == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    // SPI bus and device init
    spi_bus_config_t buscfg = {};

    buscfg.mosi_io_num = PIN_SPI_MOSI;
    buscfg.miso_io_num = PIN_SPI_MISO;
    buscfg.sclk_io_num = PIN_SPI_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Init W5500 mac and phy esp-idf layers
    spi_device_interface_config_t spi_devcfg = {};

    spi_devcfg.mode = 0;
    spi_devcfg.clock_speed_hz = 25 * 1000 * 1000;
    spi_devcfg.spics_io_num = PIN_SPI_CS;
    spi_devcfg.queue_size = 20;
   
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &spi_devcfg);
    w5500_config.base.int_gpio_num = PIN_W5500_INT;
    
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 0;
    phy_config.reset_gpio_num = -1;

    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    // Config ethernet driver and local MAC-adress from ESP32-device
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    uint8_t local_mac[6];
    get_local_mac(local_mac);
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, local_mac));

    // Init esp32 network interface
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    ESP_LOGI(TAG, "Waiting for IP address...");
    if (xSemaphoreTake(got_ip_sem, portMAX_DELAY) != pdTRUE) 
    {
        ESP_LOGE(TAG, "Failed to get IP address");
        return;
    }

    G_plnr = new MotionPlanner();
    G_plnr->init();

    xTaskCreate(udp_server_task, "udp_server", 8192, NULL, 5, NULL);
    xTaskCreate(uart_grbl_task, "uart_grbl", 4096, NULL, 5, NULL);
    xTaskCreate(planner_task, "planner", 4096, NULL, 4, NULL);

    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
