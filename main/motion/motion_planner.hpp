#pragma once

#include "motion.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

class MotionPlanner 
{
    static constexpr float JUNCTION_ACCELERATION = 100.0f;

private:
    mpMoveMaster_t  _mm;    // фактическая позиция и параметры движения
    mpMoveRuntime_t _mr;    // исполнение запланированного движения
    mpBuf_t         _bf;    // планируемое движение
    MATH::Vector2D<float>  _prev_unit;

    Axis _axes[AXES]; 

public:
    SemaphoreHandle_t _bf_mutex = nullptr;

private:
    SemaphoreHandle_t _motion_complete_sem = nullptr;
    esp_timer_handle_t _motion_timer = nullptr;
    bool _timer_running = false;

    static void timer_callback(void* arg);
    void start_timer();
    void stop_timer();

    void calculate_move_times(const MATH::Vector2D<float>& axis_length);
    float get_junction_vmax(const MATH::Vector2D<float>& a_unit, const MATH::Vector2D<float>& b_unit);
    void init_forward_diffs(float Vi, float Vt);
    void log_promt();
    void log_out(int T_ms, int dt_ms);
    stat_t exec_segment(void);
    stat_t exec_tail(void); 
    stat_t exec_cruise(void);
    stat_t exec_head(void);

public:  

    MotionPlanner();
    ~MotionPlanner();

    bool init();
    bool plan_linear_move(float target_x, float target_y, float feed_rate);
    stat_t execute_move(); // Call from periodic timer.

    bool is_busy() const noexcept { return _mr.block_state == BLOCK_RUNNING; }
    bool is_idle() const noexcept { return _mr.block_state == BLOCK_IDLE; }
    bool is_zero_move(float target_x, float target_y, const float* current_x, const float* current_y) const noexcept;

    bool wait_for_completion(TickType_t ticks = portMAX_DELAY) 
    {
        if (_motion_complete_sem != nullptr) 
        {
            return xSemaphoreTake(_motion_complete_sem, ticks) == pdTRUE;
        }
        return false;
    }

    float get_target_velocity(const float Vi, const float L) 
    {
        return Vi + (L * _bf.jerk / _bf.cbrt_jerk) / 2.0f;

    }

    float get_target_length(const float Vi, const float Vf) 
    {
        return (Vf - Vi) * _bf.cbrt_jerk * 2.0f / _bf.jerk;

    }

    static inline float get_uSec(float minutes) { return minutes * MICROSECONDS_PER_MINUTE; }

    const char* block_state_to_str(blockState_t state) const noexcept;

    const char* section_to_str(moveSection_t section) const noexcept;

    const char* section_state_to_str(sectionState_t state) const noexcept;

    void track_motion_states() const noexcept;
};

extern MotionPlanner* G_plnr;