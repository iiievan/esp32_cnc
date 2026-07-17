idf_component_register(
    SRCS 
        "udp_server.c"
        "app_tasks.c"
        "grbl_queue.c"
        "grbl_parser.c"       
    PRIV_REQUIRES 
        esp_eth 
        esp_netif
        driver
        esp_driver_uart
        nvs_flash
        motion
    INCLUDE_DIRS "."
)


