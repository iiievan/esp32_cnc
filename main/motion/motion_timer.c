#include "motion.h"
#include "esp_timer.h"


esp_timer_handle_t motion_timer = NULL;

static void motion_timer_callback(void *arg) 
{
    if (xSemaphoreTake(bf_mutex, 0) == pdTRUE) 
    {
        mp_exec_aline(&bf); 

        xSemaphoreGive(bf_mutex);
    }
    else
    {
#ifdef DBG_PLANNER_LOG
        ESP_LOGW("TIMER", "Database busy, skipping tick");
#endif       
    }
}

void start_motion_timer(void) 
{
    if (motion_timer == NULL) 
    {
        esp_timer_create_args_t timer_args = 
        {
            .callback = &motion_timer_callback,
            .arg = NULL,
            .name = "motion_timer"
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motion_timer));
        // Запускаем с периодом 5 мс (200 Гц)
        ESP_ERROR_CHECK(esp_timer_start_periodic(motion_timer, 5000));
    }
}

void stop_motion_timer(void) 
{
    if (motion_timer) 
    {
        esp_timer_stop(motion_timer);
        esp_timer_delete(motion_timer);
        motion_timer = NULL;
    }
}
