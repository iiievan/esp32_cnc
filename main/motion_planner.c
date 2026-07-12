#include "motion_planner.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"


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

// --------------------------------------------------------------------------
// Инициализация
// --------------------------------------------------------------------------
void mp_init(void) 
{
    memset(&mm, 0, sizeof(mm));
    memset(&mr, 0, sizeof(mr));
    memset(&bf, 0, sizeof(bf));

    // reset start position to zero
    mm.position[0] = 0.0; // X
    mm.position[1] = 0.0; // Y
    
    // Инициализация параметров осей
    for (int i = 0; i < AXES; i++) 
    {
        axis_params[i].recip_jerk = 1.0 / axis_params[i].jerk_max;
    }
    
    ESP_LOGI(TAG, "Motion controller initialized");
}

float mp_get_target_velocity(const float Vi, const float L, const mpBuf_t *bf) 
{
    float J = bf->jerk;
    float Vf = Vi + (L * J / bf->cbrt_jerk) / 2.0; // упрощённая формула
    // Более точная формула из TinyG:
    // Vf = Vi + (L / bf->length) * bf->delta_vmax;
    return Vf;
}

float mp_get_target_length(const float Vi, const float Vf, const mpBuf_t *bf) 
{
    float J = bf->jerk;
    float L = (Vf - Vi) * bf->cbrt_jerk * 2.0 / J;
    return L;
}

static void _calc_move_times(mpBuf_t *bf, const float axis_length[], const float axis_square[]) 
{
    float xyz_time = 0;
    float max_time = 0;
    float tmp_time;
    
    bf->minimum_time = 1e9;
    
    // Время по feedrate (для линейного движения)
    float length = bf->length;
    if (length > 0) 
    {
        xyz_time = length / bf->feed_rate; // время в минутах
    }
    
    // Время, ограниченное максимальной скоростью осей
    for (int axis = 0; axis < AXES; axis++) 
    {
        if (fabs(axis_length[axis]) > 0) 
        {
            tmp_time = fabs(axis_length[axis]) / 100.0; // max velocity = 100 mm/min (настроить)
            max_time = max(max_time, tmp_time);
            bf->minimum_time = min(bf->minimum_time, tmp_time);
        }
    }
    
    // Выбираем максимальное время
    bf->move_time = max(xyz_time, max_time);
}

// Вычисление максимальной скорости на стыке (junction velocity)
static float _get_junction_vmax(const float a_unit[], const float b_unit[]) 
{
    float costheta = 0;
    for (int i = 0; i < AXES; i++) {
        costheta -= a_unit[i] * b_unit[i];
    }
    
    if (costheta < -0.99) return 10000000; // прямая линия
    if (costheta > 0.99) return 0;         // разворот
    
    float a_delta = 0, b_delta = 0;
    for (int i = 0; i < AXES; i++) {
        a_delta += square(a_unit[i] * axis_params[i].junction_dev);
        b_delta += square(b_unit[i] * axis_params[i].junction_dev);
    }
    
    float delta = (sqrt(a_delta) + sqrt(b_delta)) / 2;
    float sintheta_over2 = sqrt((1 - costheta) / 2);
    float radius = delta * sintheta_over2 / (1 - sintheta_over2);
    float velocity = sqrt(radius * 100.0); // junction_acceleration = 100 mm/s²
    return velocity;
}

bool mp_aline(float target_x, float target_y, float feed_rate) 
{
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
    
    bf.length = sqrt(length_square);
    if (fp_ZERO(bf.length)) 
    {
        ESP_LOGW(TAG, "Zero length move");
        return false;
    }
    
    // Устанавливаем feed rate
    bf.feed_rate = feed_rate; // мм/мин
    
    // Вычисляем время движения
    _calc_move_times(&bf, axis_length, axis_square);
    
    // Вычисляем единичный вектор и jerk
    float maxC = 0;
    float recip_L2 = 1 / length_square;
    
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
    bf.recip_jerk = 1 / bf.jerk;
    bf.cbrt_jerk = cbrt(bf.jerk);
    
    // Устанавливаем скорости
    bf.cruise_vmax = bf.length / bf.move_time; 
    
    // Junction velocity (пока без учёта предыдущего движения)
    float junction_velocity = _get_junction_vmax(bf.unit, bf.unit); // упрощённо
    
    bf.entry_vmax = min3(bf.cruise_vmax, junction_velocity, 1000.0f);
    bf.delta_vmax = mp_get_target_velocity(0, bf.length, &bf);
    bf.exit_vmax = min3(bf.cruise_vmax, (bf.entry_vmax + bf.delta_vmax), 1000.0f);
    bf.braking_velocity = bf.delta_vmax;

    // Вычисляем длины сегментов разгона, круиза и торможения
    float total_length = bf.length;
    float v_entry = bf.entry_velocity;
    float v_cruise = bf.cruise_velocity;
    float v_exit = bf.exit_velocity;

    // Длина разгона (HEAD)
    bf.head_length = mp_get_target_length(v_entry, v_cruise, &bf);
    // Длина торможения (TAIL)
    bf.tail_length = mp_get_target_length(v_cruise, v_exit, &bf);
    // Длина круиза (BODY) - остаток
    bf.body_length = total_length - bf.head_length - bf.tail_length;

    // Если тело отрицательное, значит разгон и торможение перекрываются
    if (bf.body_length < 0) 
    {
        // Упрощенно: убираем круиз, разгон сразу переходит в торможение
        bf.body_length = 0;
        // Пересчитываем head и tail пропорционально
        float total_accel = bf.head_length + bf.tail_length;
        if (total_accel > 0) 
        {
            bf.head_length = (bf.head_length / total_accel) * total_length;
            bf.tail_length = total_length - bf.head_length;
        }
    }

    float total_move_time = bf.move_time * 60.0; // минуты -> секунды
    mr.segments = ceil(total_move_time / (NOM_SEGMENT_USEC / 1000000.0));
    if (mr.segments < 10) mr.segments = 10; // минимум 10 сегментов
    
    // Инициализируем runtime
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
    
    ESP_LOGI(TAG, "Move planned: target=(%.2f, %.2f), length=%.2f, time=%.3f min, jerk=%.2f, segments=%.0f", 
             target_x, target_y, bf.length, bf.move_time, bf.jerk, mr.segments);
    
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
    
    float h = 1.0f / (mr.segments);
    float Ah_5 = A * h * h * h * h * h;
    float Bh_4 = B * h * h * h * h;
    float Ch_3 = C * h * h * h;
    
    mr.forward_diff_5 = (121.0f/16.0f)*Ah_5 + 5.0f*Bh_4 + (13.0f/4.0f)*Ch_3;
    mr.forward_diff_4 = (165.0f/2.0f)*Ah_5 + 29.0f*Bh_4 + 9.0f*Ch_3;
    mr.forward_diff_3 = 255.0f*Ah_5 + 48.0f*Bh_4 + 6.0f*Ch_3;
    mr.forward_diff_2 = 300.0f*Ah_5 + 24.0f*Bh_4;
    mr.forward_diff_1 = 120.0f*Ah_5;
    
    // Начальная скорость в середине первого сегмента
    float half_h = h / 2.0;
    float half_Ch_3 = C * half_h * half_h * half_h;
    float half_Bh_4 = B * half_h * half_h * half_h * half_h;
    float half_Ah_5 = A * half_h * half_h * half_h * half_h * half_h;
    mr.segment_velocity = half_Ah_5 + half_Bh_4 + half_Ch_3 + Vi;
}

// Генерация одного сегмента (упрощенная, без энкодеров и кинематики)
static stat_t _exec_aline_segment(void) 
{
    ESP_LOGI(TAG, "DEBUG: seg_count=%d, segments=%.0f, vel=%.2f, time=%.6f", 
             mr.segment_count, mr.segments, mr.segment_velocity, mr.segment_time);
             
    if (--mr.segment_count == 0) 
    {
        // Последний сегмент — доводим до точной цели
        mr.position[0] = mr.target[0];
        mr.position[1] = mr.target[1];
        return STAT_OK;
    }
    
    // Вычисляем длину сегмента
    const float segment_length = mr.segment_velocity * mr.segment_time;
    
    // Вычисляем новую позицию по единичному вектору
    const float pos_x = mr.position[0] + mr.unit[0] * segment_length;
    const float pos_y = mr.position[1] + mr.unit[1] * segment_length;
    
    mr.position[0] = pos_x;
    mr.position[1] = pos_y;
    
    // Логирование (каждые 10%)
    static int last_log = -1;
    int progress = (int)((mr.segment_count / mr.segments) * 100.0f);
    
    if (progress != last_log) 
    {
        last_log = progress;
        ESP_LOGI(TAG, "Progress: %d%% pos=(%.3f, %.3f) vel=%.2f", 
                 progress, pos_x, pos_y, mr.segment_velocity * 60.0f);
    }
    
    return STAT_EAGAIN; // Еще есть сегменты
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
        
        const float move_time = 2 * mr.head_length / (mr.entry_velocity + mr.cruise_velocity);
        mr.segments = ceil(uSec(move_time) / NOM_SEGMENT_USEC);
        if (mr.segments < 1) mr.segments = 1;
        mr.segment_time = move_time / mr.segments;
        
        _init_forward_diffs(mr.entry_velocity, mr.cruise_velocity);
        mr.segment_count = (uint32_t)mr.segments;
        
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
            if (fp_ZERO(mr.body_length) && 
                fp_ZERO(mr.tail_length)) 
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
        mr.segments = ceil(uSec(move_time) / NOM_SEGMENT_USEC);
        if (mr.segments < 1) mr.segments = 1;
        mr.segment_time = move_time / mr.segments;
        mr.segment_velocity = mr.cruise_velocity;
        mr.segment_count = (uint32_t)mr.segments;
        
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
        
        const float move_time = 2 * mr.tail_length / (mr.cruise_velocity + mr.exit_velocity);
        mr.segments = ceil(uSec(move_time) / NOM_SEGMENT_USEC);
        if (mr.segments < 1) mr.segments = 1;
        mr.segment_time = move_time / mr.segments;
        
        _init_forward_diffs(mr.cruise_velocity, mr.exit_velocity);
        mr.segment_count = (uint32_t)mr.segments;
        
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
stat_t mp_exec_aline(mpBuf_t *bf) 
{
    if (bf->block_state == BLOCK_IDLE) 
    {
        return STAT_NOOP;
    }
    
    // Инициализация нового движения
    if (mr.block_state == BLOCK_IDLE) 
    {
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
		copy_vector(mr.target, bf->target);			// save the final target of the move
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

        // ВАЖНО: Обновляем позицию планировщика
        for (int i = 0; i < AXES; i++) {
            mm.position[i] = mr.position[i];
        }

        stop_motion_timer();

        ESP_LOGI(TAG, "Move completed at (%.2f, %.2f)", mr.position[0], mr.position[1]);
    }
    
    return status;
}

// --------------------------------------------------------------------------
// Таймер для генерации шагов (используем esp_timer)
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
        // Запускаем с периодом 5 мс (200 Гц) - для теста
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
// Тестовая функция для рисования круга.
// --------------------------------------------------------------------------
void mp_test_circle(void) 
{
    // Рисуем круг из 36 точек 
    float radius = 10.0;
    int segments = 36;
    
    for (int i = 0; i <= segments; i++) 
    {
        float angle = 2 * M_PI * i / segments;
        float x = radius * cos(angle);
        float y = radius * sin(angle);
        mp_aline(x, y, 100.0); // feed_rate = 100 мм/мин
        vTaskDelay(pdMS_TO_TICKS(100)); // задержка для визуализации
    }
    
    // Возврат в центр
    mp_aline(0, 0, 100.0);
}