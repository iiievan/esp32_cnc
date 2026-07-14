#include "motion_planner.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

//#define DBG_LOG

static const char *TAG = "MOTION_CTRL";

// Параметры оси (пример для X и Y)
axis_param_t axis_params[AXES] = 
{
    AXIS_PARAM_DEFAULT(), // X
    AXIS_PARAM_DEFAULT()  // Y
};

// Глобальные переменные
mpMoveMaster_t      mm = MPMOVE_MASTER_INIT();
mpMoveRuntime_t     mr = MPMOVE_RUNTIME_INIT();
mpBuf_t             bf = MPBUF_INIT();   

// Предыдущий unit-вектор для расчета junction_velocity
static float prev_unit[AXES] = {0.0f, 0.0f};

// --------------------------------------------------------------------------
// Инициализация
// --------------------------------------------------------------------------
void mp_init(void) 
{
    memset(&mm, 0, sizeof(mm));
    memset(&mr, 0, sizeof(mr));
    memset(&bf, 0, sizeof(bf));
    memset(prev_unit, 0, sizeof(prev_unit));

    // reset start position to zero
    mm.position[0] = 0.0; // X
    mm.position[1] = 0.0; // Y
    
    // Инициализация параметров осей
    for (int i = 0; i < AXES; i++) 
    {
        axis_params[i].recip_jerk = 1.0f / axis_params[i].jerk_max;
    }
    
    ESP_LOGI(TAG, "Motion controller initialized");
}

float mp_get_target_velocity(const float Vi, const float L, const mpBuf_t *bf) 
{
    float J = bf->jerk;
    float Vf = Vi + (L * J / bf->cbrt_jerk) / 2.0f;
    return Vf;
}

float mp_get_target_length(const float Vi, const float Vf, const mpBuf_t *bf) 
{
    float J = bf->jerk;
    float L = (Vf - Vi) * bf->cbrt_jerk * 2.0f / J;
    return L;
}

static void _calc_move_times(mpBuf_t *bf, const float axis_length[], const float axis_square[]) 
{
    float xyz_time = 0;
    float max_time = 0;
    float tmp_time;
    
    bf->minimum_time = 8675309.0f; // большое число
    
    // Время по feedrate (для линейного движения)
    float length = bf->length;
    if (length > 0) 
    {
        xyz_time = length / bf->feed_rate; // время в минутах
    }
    
    for (int axis = 0; axis < AXES; axis++) 
    {
        if (fabs(axis_length[axis]) > 0) 
        {
            // Используем feed_rate как максимальную скорость для расчета времени
            tmp_time = fabs(axis_length[axis]) / bf->feed_rate;
            max_time = max(max_time, tmp_time);
            bf->minimum_time = min(bf->minimum_time, tmp_time);
        }
    }
    
    bf->move_time = max(xyz_time, max_time);
    
    // Проверка на короткие движения (как в TinyG)
    if (bf->move_time < MIN_BLOCK_TIME) 
    {
        float delta_velocity = powf(length, 0.66666666f) * mm.cbrt_jerk;
        float entry_velocity = 0.0f;
        // Для первого движения entry_velocity = 0
        float move_time = (2.0f * length) / (2.0f * entry_velocity + delta_velocity);
        if (move_time < MIN_BLOCK_TIME) 
        {
            // Слишком короткое движение — пропускаем
            bf->move_time = MIN_BLOCK_TIME;
        } 
        else 
        {
            bf->move_time = move_time;
        }
    }
}

// --------------------------------------------------------------------------
// _get_junction_vmax() - расчет максимальной скорости на стыке
// --------------------------------------------------------------------------
static float _get_junction_vmax(const float a_unit[], const float b_unit[]) 
{
    float costheta = 0;
    for (int i = 0; i < AXES; i++) {
        costheta -= a_unit[i] * b_unit[i];
    }
    
    if (costheta < -0.99f) return 10000000.0f; // прямая линия
    if (costheta > 0.99f) return 0.0f;         // разворот
    
    float a_delta = 0, b_delta = 0;
    for (int i = 0; i < AXES; i++) {
        a_delta += square(a_unit[i] * axis_params[i].junction_dev);
        b_delta += square(b_unit[i] * axis_params[i].junction_dev);
    }
    
    float delta = (sqrtf(a_delta) + sqrtf(b_delta)) / 2.0f;
    float sintheta_over2 = sqrtf((1.0f - costheta) / 2.0f);
    float radius = delta * sintheta_over2 / (1.0f - sintheta_over2);
    float velocity = sqrtf(radius * 100.0f); // junction_acceleration = 100 mm/s²
    return velocity;
}

// --------------------------------------------------------------------------
// mp_aline() - главная функция планирования
// --------------------------------------------------------------------------
bool mp_aline(float target_x, float target_y, float feed_rate) 
{
    ESP_LOGI(TAG, "=== mp_aline() START: target=(%.2f, %.2f), feed=%.2f ===", 
             target_x, target_y, feed_rate);
    ESP_LOGI(TAG, "  mm.position = (%.2f, %.2f)", mm.position[0], mm.position[1]);

    if (mr.block_state == BLOCK_RUNNING) 
    {
        ESP_LOGW(TAG, "Planner busy, cannot start new move");
        return false;
    }

    // Очищаем буфер
    memset(&bf, 0, sizeof(mpBuf_t));
    bf.block_state = BLOCK_RUNNING; 
    
    // Вычисляем длины по осям
    float axis_length[AXES];
    float axis_square[AXES];
    float length_square = 0;
    
    axis_length[0] = target_x - mm.position[0]; // X
    axis_length[1] = target_y - mm.position[1]; // Y
    
    for (int i = 0; i < AXES; i++) 
    {
        axis_square[i] = square(axis_length[i]);
        length_square += axis_square[i];
    }
    
    bf.length = sqrtf(length_square);

    ESP_LOGI(TAG, "  bf.length = %.4f", bf.length);
    
    if (fp_ZERO(bf.length)) 
    {
        ESP_LOGW(TAG, "Zero length move");
        return false;
    }
    
    // Устанавливаем feed rate
    bf.feed_rate = feed_rate; // мм/мин
    
    // Вычисляем время движения
    _calc_move_times(&bf, axis_length, axis_square);

    ESP_LOGI(TAG, "  bf.move_time = %.6f min", bf.move_time);
    
    // Вычисляем единичный вектор и jerk
    float maxC = 0;
    float recip_L2 = 1.0f / length_square;
    
    for (int axis = 0; axis < AXES; axis++) 
    {
        if (fabs(axis_length[axis]) > 0) 
        {
            bf.unit[axis] = axis_length[axis] / bf.length;
            float C = axis_square[axis] * recip_L2 * axis_params[axis].recip_jerk;
            if (C > maxC) 
            {
                maxC = C;
                bf.jerk_axis = axis;
            }
        }
    }
    
    bf.jerk = axis_params[bf.jerk_axis].jerk_max * JERK_MULTIPLIER / fabs(bf.unit[bf.jerk_axis]);
    bf.recip_jerk = 1.0f / bf.jerk;
    bf.cbrt_jerk = cbrtf(bf.jerk);

    ESP_LOGI(TAG, "  bf.jerk = %.2f, axis=%d", bf.jerk, bf.jerk_axis);
    
    // Устанавливаем скорости
    bf.cruise_vmax = bf.length / bf.move_time; 
    
    // Junction velocity с учетом предыдущего движения
    float junction_velocity = _get_junction_vmax(prev_unit, bf.unit);
    // Сохраняем текущий unit для следующего движения
    copy_vector(prev_unit, bf.unit);
    
    // exact_stop = большое число (нет точной остановки)
    float exact_stop = 8675309.0f;
    bf.replannable = true;
    
    bf.entry_vmax = min3(bf.cruise_vmax, junction_velocity, exact_stop);
    bf.delta_vmax = mp_get_target_velocity(0.0f, bf.length, &bf);
    bf.exit_vmax = min3(bf.cruise_vmax, (bf.entry_vmax + bf.delta_vmax), exact_stop);
    bf.braking_velocity = bf.delta_vmax;
    bf.entry_velocity = bf.entry_vmax;
    bf.cruise_velocity = bf.cruise_vmax;
    bf.exit_velocity = 0.0f;


    ESP_LOGI(TAG, "  speeds: entry=%.2f, cruise=%.2f, exit=%.2f", 
             bf.entry_velocity, bf.cruise_velocity, bf.exit_velocity);

    float total_length = bf.length;
    float v_entry = bf.entry_velocity;
    float v_cruise = bf.cruise_velocity;
    float v_exit = bf.exit_velocity;


    bf.head_length = mp_get_target_length(v_entry, v_cruise, &bf);
    bf.tail_length = mp_get_target_length(v_cruise, v_exit, &bf);
    if (bf.tail_length < 0) bf.tail_length = -bf.tail_length;
    bf.body_length = total_length - bf.head_length - bf.tail_length;

    ESP_LOGI(TAG, "  lengths: head=%.4f, body=%.4f, tail=%.4f", 
             bf.head_length, bf.body_length, bf.tail_length);

    // Если сумма head + tail больше общей длины — пропорционально уменьшаем
    if (bf.body_length < 0) 
    {
        float total_accel = bf.head_length + bf.tail_length;
        if (total_accel > 0) 
        {
            bf.head_length = (bf.head_length / total_accel) * total_length;
            bf.tail_length = (bf.tail_length / total_accel) * total_length;
            bf.body_length = 0.0f;
        }
        ESP_LOGI(TAG, "  corrected: head=%.4f, body=%.4f, tail=%.4f", 
                 bf.head_length, bf.body_length, bf.tail_length);
    }

    float total_move_time = bf.move_time * 60.0f;
    mr.segments = ceilf(total_move_time / (NOM_SEGMENT_USEC / 1000000.0f));
    if (mr.segments < 10) mr.segments = 10;    

    ESP_LOGI(TAG, "  mr.segments = %.0f", mr.segments);

    mr.block_state = BLOCK_RUNNING;
    mr.section = SECTION_HEAD;
    mr.section_state = SECTION_NEW;
    
    for (int i = 0; i < AXES; i++) 
    {
        mr.unit[i] = bf.unit[i];
        mr.target[i] = mm.position[i] + axis_length[i];
        mr.position[i] = mm.position[i];
    }
    
    mr.head_length = bf.head_length;
    mr.body_length = bf.body_length;
    mr.tail_length = bf.tail_length;
    
    mr.entry_velocity = bf.entry_velocity;
    mr.cruise_velocity = bf.cruise_velocity;
    mr.exit_velocity = bf.exit_velocity;
    
    // Обновляем позицию планировщика
    for (int i = 0; i < AXES; i++) 
    {
        mm.position[i] = mr.target[i];
    }
 
#ifdef DBG_LOG
    ESP_LOGI(TAG, "=== mp_aline() END: success ===");
#else
    ESP_LOGI(TAG, "Move planned: target=(%.2f, %.2f), length=%.2f, time=%.3f min, jerk=%.2f, segments=%.0f", 
             target_x, target_y, bf.length, bf.move_time, bf.jerk, mr.segments);
#endif
   
    // Запускаем генерацию шагов (в таймере)
    // Здесь вызываем функцию, которая инициализирует таймер
    start_motion_timer();
    
    return true;
}

// --------------------------------------------------------------------------
// mp_exec_aline() - генерация шагов в таймере
// --------------------------------------------------------------------------

// Вспомогательная функция: вычисление времени в микросекундах
static inline float uSec(float minutes) 
{
    return minutes * MICROSECONDS_PER_MINUTE;
}

static stat_t _exec_aline_tail(void); 
static stat_t _exec_aline_body(void);
static stat_t _exec_aline_head(void);
static stat_t _exec_aline_segment(void); 

// Инициализация forward differences (из TinyG)
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
 
#ifdef DBG_LOG
    ESP_LOGI(TAG, "  _init_forward_diffs: Vi=%.2f, Vt=%.2f, segments=%.0f, h=%.6f", 
             Vi, Vt, mr.segments, h);
    ESP_LOGI(TAG, "    forward_diff: d1=%.6f, d2=%.6f, d3=%.6f, d4=%.6f, d5=%.6f",
             mr.forward_diff_1, mr.forward_diff_2, mr.forward_diff_3, 
             mr.forward_diff_4, mr.forward_diff_5);
    ESP_LOGI(TAG, "    segment_velocity = %.6f", mr.segment_velocity);
#endif
}

static stat_t _exec_aline_segment(void) 
{
    // Уменьшаем счётчик сегментов
    mr.segment_count--;
    
    // Если это последний сегмент и мы в конце секции — используем waypoint
    if (mr.segment_count == 0 && mr.section_state == SECTION_2nd_HALF) {
        copy_vector(mr.position, mr.waypoint[mr.section]);
        return STAT_OK;
    }
    
    // Иначе вычисляем позицию через segment_length
    float segment_length = mr.segment_velocity * mr.segment_time;
    float pos_x = mr.position[0] + mr.unit[0] * segment_length;
    float pos_y = mr.position[1] + mr.unit[1] * segment_length;
    
    mr.position[0] = pos_x;
    mr.position[1] = pos_y;
    
    // Логирование (каждые 10%)
    int total_segments = (int)mr.segments;
    int progress = (int)(((total_segments - mr.segment_count) / (float)total_segments) * 100.0f);
    static int last_log = -1;
    if (progress != last_log && (progress % 10 == 0 || progress == 99)) {
        last_log = progress;
        ESP_LOGI(TAG, "Progress: %d%% pos=(%.3f, %.3f) vel=%.2f", 
                 progress, pos_x, pos_y, mr.segment_velocity * 60.0f);
    }
    
    return STAT_EAGAIN;
}

// HEAD секция (разгон)
static stat_t _exec_aline_head(void) 
{
    if (mr.section_state == SECTION_NEW) 
    {
        if (fp_ZERO(mr.head_length)) 
        {
            mr.section = SECTION_BODY;
            return _exec_aline_body();
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

// BODY секция (круиз)
static stat_t _exec_aline_body(void) 
{
    if (mr.section_state == SECTION_NEW) 
    {
        if (fp_ZERO(mr.body_length)) 
        {
            mr.section = SECTION_TAIL;
            return _exec_aline_tail();
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

// TAIL секция (торможение)
static stat_t _exec_aline_tail(void) 
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

// --------------------------------------------------------------------------
// ГЛАВНАЯ ФУНКЦИЯ mp_exec_aline()
// --------------------------------------------------------------------------

static const char* block_state_to_str(blockState_t state) 
{
    switch(state) {
        case BLOCK_IDLE: return "IDLE";
        case BLOCK_INITIALIZING: return "INITIALIZING";
        case BLOCK_RUNNING: return "RUNNING";
        default: return "UNKNOWN";
    }
}

static const char* section_to_str(moveSection_t section) 
{
    switch(section) {
        case SECTION_NA: return "N/A";
        case SECTION_HEAD: return "HEAD";
        case SECTION_BODY: return "BODY";
        case SECTION_TAIL: return "TAIL";
        default: return "UNKNOWN";
    }
}

static const char* section_state_to_str(sectionState_t state) 
{
    switch(state) {
        case SECTION_OFF: return "OFF";
        case SECTION_NEW: return "NEW";
        case SECTION_1st_HALF: return "1st_HALF";
        case SECTION_2nd_HALF: return "2nd_HALF";
        default: return "UNKNOWN";
    }
}

stat_t mp_exec_aline(mpBuf_t *bf) 
{
#ifdef DBG_LOG
    ESP_LOGI(TAG, "=== mp_exec_aline() START: bf->block_state=%d ===", bf->block_state);
#endif  
    if (bf->block_state == BLOCK_IDLE) 
    {
#ifdef DBG_LOG
        ESP_LOGI(TAG, "  bf is IDLE, returning STAT_NOOP");
#endif
        return STAT_NOOP;
    }
    
    // Инициализация нового движения
    if (mr.block_state == BLOCK_IDLE) 
    {
#ifdef DBG_LOG
        ESP_LOGI(TAG, "  Initializing new movement");
#endif
        bf->replannable = false;
        bf->block_state = BLOCK_RUNNING;
        mr.block_state = BLOCK_INITIALIZING;
        mr.section = SECTION_HEAD;
        mr.section_state = SECTION_NEW;
        
        mr.head_length = bf->head_length;
        mr.body_length = bf->body_length;
        mr.tail_length = bf->tail_length;
        
        mr.entry_velocity = bf->entry_velocity;
        mr.cruise_velocity = bf->cruise_velocity;
        mr.exit_velocity = bf->exit_velocity;

        copy_vector(mr.unit, bf->unit);
        copy_vector(mr.target, bf->target);
        copy_vector(mr.axis_flags, bf->axis_flags);          
        
        // Вычисляем waypoints для коррекции позиции
        for (int i = 0; i < AXES; i++) 
        {
            mr.waypoint[SECTION_HEAD][i] = mr.position[i] + mr.unit[i] * mr.head_length;
            mr.waypoint[SECTION_BODY][i] = mr.position[i] + mr.unit[i] * (mr.head_length + mr.body_length);
            mr.waypoint[SECTION_TAIL][i] = mr.position[i] + mr.unit[i] * (mr.head_length + mr.body_length + mr.tail_length);
        }
        
        mr.block_state = BLOCK_RUNNING;
    }
    
    // Диспетчер секций
    stat_t status = STAT_OK;
    if (mr.section == SECTION_HEAD) 
    {
        status = _exec_aline_head();
    } 
    else if (mr.section == SECTION_BODY) 
    {
        status = _exec_aline_body();
    } 
    else if (mr.section == SECTION_TAIL) 
    {
        status = _exec_aline_tail();
    } 
    else 
    {
        ESP_LOGE(TAG, "Invalid section");
        return STAT_INTERNAL_ERROR;
    }
    
    // Завершение движения
    if (status == STAT_OK) 
    {        
        mr.section_state = SECTION_OFF;
        mr.block_state = BLOCK_IDLE;
        bf->block_state = BLOCK_IDLE;

        // Обновляем позицию планировщика
        for (int i = 0; i < AXES; i++) 
        {
            mm.position[i] = mr.position[i];
        }

        stop_motion_timer();

        ESP_LOGI(TAG, "Move completed at (%.2f, %.2f)", mr.position[0], mr.position[1]);
    }

    static blockState_t old_block_state = BLOCK_IDLE;
    static moveSection_t old_section = SECTION_NA;
    static sectionState_t old_section_state = SECTION_OFF;

    if (mr.block_state != old_block_state) 
    {
        ESP_LOGI(TAG, "Block state: %s -> %s", 
                 block_state_to_str(old_block_state), 
                 block_state_to_str(mr.block_state));
        old_block_state = mr.block_state;
    }

    if (mr.section != old_section) 
    {
        ESP_LOGI(TAG, "Section: %s -> %s", 
                 section_to_str(old_section), 
                 section_to_str(mr.section));
        old_section = mr.section;
    }

    if (mr.section_state != old_section_state) 
    {
        ESP_LOGI(TAG, "Section state: %s -> %s", 
                 section_state_to_str(old_section_state), 
                 section_state_to_str(mr.section_state));
        old_section_state = mr.section_state;
    }

#ifdef DBG_LOG
    ESP_LOGI(TAG, "=== mp_exec_aline() END: status=%d ===", status);
#endif
    return status;
}

// --------------------------------------------------------------------------
// Таймер для генерации шагов (esp_timer)
// --------------------------------------------------------------------------
esp_timer_handle_t motion_timer = NULL;

static void motion_timer_callback(void *arg) 
{
    mp_exec_aline(&bf);
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

// --------------------------------------------------------------------------
// Тестовая функция для рисования круга
// --------------------------------------------------------------------------
void mp_test_circle(void) 
{
    float radius = 10.0f;
    int segments = 36;
    
    for (int i = 0; i <= segments; i++) 
    {
        float angle = 2.0f * M_PI * i / segments;
        float x = radius * cosf(angle);
        float y = radius * sinf(angle);
        mp_aline(x, y, 100.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Возврат в центр
    mp_aline(0, 0, 100.0);
}