idf_component_register(
    SRCS 
        "udp_server.cpp"
        "app_tasks.cpp"
        "grbl_queue.cpp"
        "grbl_parser.cpp"       
    PRIV_REQUIRES 
        esp_eth 
        esp_netif
        driver
        esp_driver_uart
        nvs_flash
        motion
    INCLUDE_DIRS "."
)


