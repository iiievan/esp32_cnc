idf_component_register(
    SRCS 
        "motion_planner.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver
        esp_timer
        freertos
)
