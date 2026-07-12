idf_component_register(
    SRCS 
        "udp_server.c"
        "app_tasks.c"
        "grbl_queue.c"
        "grbl_parser.c"
        "motion_planner.c"
    PRIV_REQUIRES 
        esp_eth 
        esp_netif
        driver
        nvs_flash
    INCLUDE_DIRS "."
)
