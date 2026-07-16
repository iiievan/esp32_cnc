#include "motion.h"
#include "esp_timer.h"


esp_timer_handle_t motion_timer = NULL;
static bool timer_running = false;

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

void motion_timer_init(void) 
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
        timer_running = false;
        ESP_LOGI("TIMER", "Motion timer created successfully");
    }
}

void start_motion_timer(void) 
{
    if (motion_timer != NULL && !timer_running) 
    {
        // Запускаем уже созданный таймер с периодом 5 мс (5000 мкс)
        ESP_ERROR_CHECK(esp_timer_start_periodic(motion_timer, 5000));
        timer_running = true;
#ifdef DBG_PLANNER_LOG
        ESP_LOGD("TIMER", "Timer started");
#endif
    }
}

void stop_motion_timer(void) 
{
    if (motion_timer != NULL && timer_running) 
    {
        // Просто останавливаем таймер, НЕ удаляя его
        ESP_ERROR_CHECK(esp_timer_stop(motion_timer));
        timer_running = false;
#ifdef DBG_PLANNER_LOG
        ESP_LOGD("TIMER", "Timer stopped");
#endif
    }
}
