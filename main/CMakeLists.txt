idf_component_register(
    SRCS 
        "udp_server.c"
        "grbl_queue.c"
        "grbl_parser.c"
    PRIV_REQUIRES 
        esp_eth 
        esp_netif
        driver
        nvs_flash
    INCLUDE_DIRS "."
)
