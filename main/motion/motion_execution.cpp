#include "motion.h"
#include "motion_timer.h"
#include "esp_timer.h"


//#define DBG_EXE_LOG (1)
static const char *TAG = "MOT_EXECUTE";

static inline void log_promt()
{
    ESP_LOGI(TAG, "MOVE START");
    ESP_LOGI(TAG, "pos_x_mm:pos_y_mm:vel_mm_min:T_ms:dt_ms");
}

static inline void log_out(int T_ms, int dt_ms)
{
    ESP_LOGI(TAG, "%.3f:%.3f:%.2f:%d:%d", mr.position.x, mr.position.y, 
                                          mr.segment_velocity * 60.0f, 
                                          T_ms, 
                                          dt_ms);
}

static stat_t _exec_tail(void); 
static stat_t _exec_cruise(void);
static stat_t _exec_head(void);
static void   _init_forward_diffs(float Vi, float Vt);

static stat_t _exec_aline_segment(void) 
{
    // Уменьшаем счётчик сегментов
    mr.segment_count--;
    
    // Если это последний сегмент и мы в конце секции — используем waypoint
    if (mr.segment_count == 0 && mr.section_state == SECTION_2nd_HALF) 
    {
        mr.gm.target = mr.waypoint[mr.section];
    } 
    else 
    {
        float segment_length = mr.segment_velocity * mr.segment_time;
		mr.gm.target = mr.position + (mr.unit * segment_length);
    }

#ifdef DBG_EXE_LOG
    static moveSection_t old_section = SECTION_NA;
    if (mr.section != old_section) 
    {
        ESP_LOGI(TAG, "Pos old:%.3f, %.3f", mr.position.x, mr.position.y);
        old_section = mr.section;
    }
#endif    
    
    mr.position = mr.gm.target; // Обновляем позицию из mr.gm.target для нового сегмента

    static int64_t last_log_time = 0;    
    const int64_t now = esp_timer_get_time();
    
    // Первый лог или прошло >= 100 мс
    if (last_log_time == 0 || (now - last_log_time) >= 100000) 
    {
        int dt_ms = (int)((now - last_log_time) / 1000);
        int T_ms = (int)(now / 1000);
        
        log_out(T_ms, dt_ms);
        
        last_log_time = now;
    }

    if (mr.segment_count == 0) return (STAT_OK);    
    return STAT_EAGAIN;
}

stat_t mp_exec_aline(mpBuf_t *bf) 
{
    if (bf->block_state == BLOCK_IDLE) 
    {
        return STAT_NOOP;
    }
    
    if (mr.section_state == SECTION_OFF) 
    {
        log_promt();

        bf->replannable = false;
        bf->block_state = BLOCK_RUNNING;
        mr.block_state = BLOCK_INITIALIZING;
        mr.section = SECTION_HEAD;
        mr.section_state = SECTION_NEW;
        mm.jerk = bf->jerk;
        
        mr.head_length = bf->head_length;
        mr.body_length = bf->body_length;
        mr.tail_length = bf->tail_length;
        
        mr.entry_velocity = bf->entry_velocity;
        mr.cruise_velocity = bf->cruise_velocity;
        mr.exit_velocity = bf->exit_velocity;

        mr.unit = bf->unit;
        mr.target = bf->gm.target;          
        
        // Вычисляем waypoints для коррекции позиции
        mr.waypoint[SECTION_HEAD] = mr.position + mr.unit * mr.head_length;
        mr.waypoint[SECTION_BODY] = mr.position + mr.unit * (mr.head_length + mr.body_length);
        mr.waypoint[SECTION_TAIL] = mr.position + mr.unit * (mr.head_length + mr.body_length + mr.tail_length);

        
        mr.block_state = BLOCK_RUNNING;
    }
    
    stat_t status = STAT_OK;
    switch(mr.section)
    {
        case SECTION_HEAD:
            status = _exec_head();
            break;

        case SECTION_BODY:
            status = _exec_cruise();
            break;

        case SECTION_TAIL:
            status = _exec_tail();
            break;
        default:
            ESP_LOGE(TAG, "Invalid section");
            return STAT_INTERNAL_ERROR;
    }
    
    // Завершаем движение если нет новой секции
    if (status != STAT_EAGAIN) 
    {    
        mr.section_state = SECTION_OFF;
        mr.block_state = BLOCK_IDLE;
        bf->block_state = BLOCK_IDLE;

        mm.position = mr.position;

        stop_motion_timer();

        ESP_LOGI(TAG, "MOVE COMPLETED AT (%.2f, %.2f)", mr.position.x, mr.position.x);
        
        if (motion_complete_sem != NULL) 
        {
            xSemaphoreGive(motion_complete_sem);
        }
    }

    void track_motion_states();

    return status;
}

static stat_t _exec_head(void) 
{
    if (mr.section_state == SECTION_NEW) 
    {
        if (fp_ZERO(mr.head_length)) 
        {
            mr.section = SECTION_BODY;
            return _exec_cruise();
        }
        
        const float move_time = 2.0f * mr.head_length / (mr.entry_velocity + mr.cruise_velocity);
        
        // ВАЖНО: пересчитываем количество сегментов для этой секции
        float segments = ceil(uSec(move_time) / NOM_SEGMENT_USEC);
        if (segments < 1) segments = 1;
        mr.segment_time = move_time / segments;
        
        // Используем локальное количество сегментов для forward differences
        mr.segments = segments;
        mr.segment_count = (uint32_t)segments;
        
        _init_forward_diffs(mr.entry_velocity, mr.cruise_velocity);
        
        if (mr.segment_time < MIN_SEGMENT_TIME) return STAT_MINIMUM_TIME_MOVE;
        
        mr.section = SECTION_HEAD;
        mr.section_state = SECTION_1st_HALF;
    }
    
    if (mr.section_state == SECTION_1st_HALF) 
    {
        if (_exec_aline_segment() == STAT_OK) 
        {
            mr.section = SECTION_BODY;
            mr.section_state = SECTION_NEW;
            return STAT_EAGAIN; 
        } 
        else 
        {
            mr.section_state = SECTION_2nd_HALF;
        }
        return STAT_EAGAIN;
    }
    
    if (mr.section_state == SECTION_2nd_HALF) 
    {
        mr.segment_velocity += mr.forward_diff_5;

        if (_exec_aline_segment() == STAT_OK) 
        {
            if (fp_ZERO(mr.body_length) && fp_ZERO(mr.tail_length)) 
            {
                return STAT_OK;                    
            }
                
            mr.section = SECTION_BODY;
            mr.section_state = SECTION_NEW;
        } 
        else 
        {
            mr.forward_diff_5 += mr.forward_diff_4;
            mr.forward_diff_4 += mr.forward_diff_3;
            mr.forward_diff_3 += mr.forward_diff_2;
            mr.forward_diff_2 += mr.forward_diff_1;
        }
    }
    return STAT_EAGAIN;
}

static stat_t _exec_cruise(void) 
{
    if (mr.section_state == SECTION_NEW) 
    {
        if (fp_ZERO(mr.body_length)) 
        {
            mr.section = SECTION_TAIL;
            return _exec_tail();
        }
        
        const float move_time = mr.body_length / mr.cruise_velocity;
        float segments = ceil(uSec(move_time) / NOM_SEGMENT_USEC);
        if (segments < 1) segments = 1;
        mr.segment_time = move_time / segments;
        mr.segments = segments;
        mr.segment_velocity = mr.cruise_velocity;
        mr.segment_count = (uint32_t)segments;
        
        if (mr.segment_time < MIN_SEGMENT_TIME) return STAT_MINIMUM_TIME_MOVE;
        
        mr.section = SECTION_BODY;
        mr.section_state = SECTION_2nd_HALF;
    }
    
    if (mr.section_state == SECTION_2nd_HALF) 
    {
        if (_exec_aline_segment() == STAT_OK) 
        {
            if (fp_ZERO(mr.tail_length)) return STAT_OK;

            mr.section = SECTION_TAIL;
            mr.section_state = SECTION_NEW;
        }
    }
    return STAT_EAGAIN;
}

static stat_t _exec_tail(void) 
{
    if (mr.section_state == SECTION_NEW) 
    {
        if (fp_ZERO(mr.tail_length)) return STAT_OK;
        
        const float move_time = 2.0f * mr.tail_length / (mr.cruise_velocity + mr.exit_velocity);
        float segments = ceil(uSec(move_time) / NOM_SEGMENT_USEC);
        if (segments < 1) segments = 1;
        mr.segment_time = move_time / segments;
        mr.segments = segments;
        mr.segment_count = (uint32_t)segments;
        
        _init_forward_diffs(mr.cruise_velocity, mr.exit_velocity);
        
        if (mr.segment_time < MIN_SEGMENT_TIME) return STAT_MINIMUM_TIME_MOVE;
        
        mr.section = SECTION_TAIL;
        mr.section_state = SECTION_1st_HALF;
    }
    
    if (mr.section_state == SECTION_1st_HALF) 
    {
        if (_exec_aline_segment() == STAT_OK) 
        {
            mr.section_state = SECTION_2nd_HALF;
            return STAT_OK;
        } 
        else 
        {
            mr.section_state = SECTION_2nd_HALF;
        }
        return STAT_EAGAIN;
    }
    
    if (mr.section_state == SECTION_2nd_HALF) 
    {
        mr.segment_velocity += mr.forward_diff_5;

        if (_exec_aline_segment() == STAT_OK) 
        {
            return STAT_OK;
        } 
        else 
        {
            mr.forward_diff_5 += mr.forward_diff_4;
            mr.forward_diff_4 += mr.forward_diff_3;
            mr.forward_diff_3 += mr.forward_diff_2;
            mr.forward_diff_2 += mr.forward_diff_1;
        }
    }
    return STAT_EAGAIN;
}

static void _init_forward_diffs(float Vi, float Vt) 
{
    if (mr.segments < 1) mr.segments = 1;

    float A = -6.0f  * Vi + 6.0f  * Vt;
    float B =  15.0f * Vi - 15.0f * Vt;
    float C = -10.0f * Vi + 10.0f * Vt;
    
    float h = 1.0f / mr.segments;
    float Ah_5 = A * h * h * h * h * h;
    float Bh_4 = B * h * h * h * h;
    float Ch_3 = C * h * h * h;
    
    mr.forward_diff_5 = (121.0f/16.0f)*Ah_5 + 5.0f*Bh_4 + (13.0f/4.0f)*Ch_3;
    mr.forward_diff_4 = (165.0f/2.0f)*Ah_5 + 29.0f*Bh_4 + 9.0f*Ch_3;
    mr.forward_diff_3 = 255.0f*Ah_5 + 48.0f*Bh_4 + 6.0f*Ch_3;
    mr.forward_diff_2 = 300.0f*Ah_5 + 24.0f*Bh_4;
    mr.forward_diff_1 = 120.0f*Ah_5;
    
    // Начальная скорость в середине первого сегмента
    float half_h = h / 2.0f;
    float half_Ch_3 = C * half_h * half_h * half_h;
    float half_Bh_4 = B * half_h * half_h * half_h * half_h;
    float half_Ah_5 = A * half_h * half_h * half_h * half_h * half_h;
    mr.segment_velocity = half_Ah_5 + half_Bh_4 + half_Ch_3 + Vi;
 
#ifdef DBG_EXE_LOG
    ESP_LOGI(TAG, "  _init_forward_diffs: Vi=%.2f, Vt=%.2f, segments=%.0f, h=%.6f", 
             Vi, Vt, mr.segments, h);
    ESP_LOGI(TAG, "    forward_diff: d1=%.6f, d2=%.6f, d3=%.6f, d4=%.6f, d5=%.6f",
             mr.forward_diff_1, mr.forward_diff_2, mr.forward_diff_3, 
             mr.forward_diff_4, mr.forward_diff_5);
    ESP_LOGI(TAG, "    segment_velocity = %.6f", mr.segment_velocity);
#endif
}