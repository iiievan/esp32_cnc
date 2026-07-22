idf_component_register(
    SRCS 
        "motion_planner.hpp"
        "motion_planner.cpp"
        "motion_utils.cpp"       
        "linear_algebra.hpp" 
    INCLUDE_DIRS "."
    REQUIRES 
        driver
        esp_timer
        freertos
)
