idf_component_register(
    SRCS 
        "motion_planner.cpp"
        "motion_execution.cpp"
        "motion_utils.cpp"
        "motion_timer.cpp"
        "linear_algebra.hpp" 
    INCLUDE_DIRS "."
    REQUIRES 
        driver
        esp_timer
        freertos
)
