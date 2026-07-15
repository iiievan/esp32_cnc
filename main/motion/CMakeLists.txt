idf_component_register(
    SRCS 
        "motion_planner.c"
        "motion_execution.c"
        "motion_utils.c"
        "motion_timer.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver
        esp_timer
        freertos
)
