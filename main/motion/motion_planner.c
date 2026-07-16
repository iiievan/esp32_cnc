#include "motion.h"
#include "motion_timer.h"


static const char *TAG = "MOT_PLAN";

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
SemaphoreHandle_t bf_mutex = NULL; // мьютекс для защиты буфера.
SemaphoreHandle_t motion_complete_sem = NULL;   // семафор для обозначения окончания движения

// Предыдущий unit-вектор для расчета junction_velocity
static float prev_unit[AXES] = {0.0f, 0.0f};

static void _calc_move_times(mpBuf_t *bf, const float axis_length[], const float axis_square[]);
static float _get_junction_vmax(const float a_unit[], const float b_unit[]);

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

    if (bf_mutex == NULL) 
    {
        bf_mutex = xSemaphoreCreateMutex();
        if (bf_mutex == NULL) 
        {
            ESP_LOGE(TAG, "Failed to create bf_mutex");
        }
    }

    if (motion_complete_sem == NULL) 
    {
        motion_complete_sem = xSemaphoreCreateBinary();
        if (motion_complete_sem == NULL) 
        {
            ESP_LOGE(TAG, "Failed to create motion_complete_sem");
        }
    }

    motion_timer_init();
    
    ESP_LOGI(TAG, "Motion controller initialized");
}

bool is_zero_move(float target_x, float target_y, const float* current_x, const float* current_y) 
{
    float cx = (current_x != NULL) ? *current_x : mm.position[0];
    float cy = (current_y != NULL) ? *current_y : mm.position[1];
    
    float dx = target_x - cx;
    float dy = target_y - cy;
    float distance = sqrtf(dx*dx + dy*dy);
    
    return (fabs(distance) < EPSILON);
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

    if (is_zero_move_default(target_x, target_y)) 
    {
        ESP_LOGW(TAG, "Zero length move to (%.2f, %.2f) - skipping", target_x, target_y);

        mm.position[0] = target_x;
        mm.position[1] = target_y;

        return true; // Возвращаем true, так как уже в этой позиции
    }

    if (xSemaphoreTake(bf_mutex, portMAX_DELAY) == pdTRUE) 
    {
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
        bf.gm.feed_rate = feed_rate; // мм/мин
        
        // Вычисляем время движения
        _calc_move_times(&bf, axis_length, axis_square);

        ESP_LOGI(TAG, "  bf.gm.move_time = %.6f min", bf.gm.move_time);
        
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
        bf.cruise_vmax = bf.length / bf.gm.move_time; 

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

        float total_move_time = bf.gm.move_time * 60.0f;
        mr.segments = ceilf(total_move_time / (NOM_SEGMENT_USEC / 1000000.0f));
        if (mr.segments < 10) mr.segments = 10;    

        ESP_LOGI(TAG, "  mr.segments = %.0f", mr.segments);

        mr.block_state = BLOCK_IDLE;
        mr.section = SECTION_HEAD;
        mr.section_state = SECTION_OFF;

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

        copy_vector(mm.position, bf.gm.target); 

        ESP_LOGI(TAG, "Move planned: target=(%.2f, %.2f), length=%.2f, time=%.3f min, jerk=%.2f, segments=%.0f", 
                 target_x, target_y, bf.length, bf.gm.move_time, bf.jerk, mr.segments);

        xSemaphoreGive(bf_mutex);   // отдаем мьютекс более высокопреоритетной задаче выполения движения stat_t mp_exec_aline(mpBuf_t *bf) 
    }
    else
    {
#ifdef DBG_PLANNER_LOG
        ESP_LOGE(TAG, "Failed to take mutex in mp_aline");
#endif
        return false;
    }
    // Запускаем генерацию шагов (в таймере)
    // Здесь вызываем функцию, которая инициализирует таймер
    start_motion_timer();
    
    return true;
}

static void _calc_move_times(mpBuf_t *bf, const float axis_length[], const float axis_square[]) 
{
    float xyz_time = 0;
    float max_time = 0;
    float tmp_time;
    
    bf->gm.minimum_time = 8675309.0f; // <-большое число
    
    // Время по feedrate (для линейного движения)
    float length = bf->length;
    if (length > 0) 
    {
        xyz_time = length / bf->gm.feed_rate; // время в минутах
    }
    
    for (int axis = 0; axis < AXES; axis++) 
    {
        if (fabs(axis_length[axis]) > 0) 
        {
            // Используем feed_rate как максимальную скорость для расчета времени
            tmp_time = fabs(axis_length[axis]) / bf->gm.feed_rate;
            max_time = max(max_time, tmp_time);
            bf->gm.minimum_time = min(bf->gm.minimum_time, tmp_time);
        }
    }
    
    bf->gm.move_time = max(xyz_time, max_time);
    
    // Проверка на короткие движения
    if (bf->gm.move_time < MIN_BLOCK_TIME) 
    {
        float delta_velocity = powf(length, 0.66666666f) * mm.cbrt_jerk;
        float entry_velocity = 0.0f;
        // Для первого движения entry_velocity = 0
        float move_time = (2.0f * length) / (2.0f * entry_velocity + delta_velocity);
        if (move_time < MIN_BLOCK_TIME) 
        {
            // Слишком короткое движение — пропускаем
            bf->gm.move_time = MIN_BLOCK_TIME;
        } 
        else 
        {
            bf->gm.move_time = move_time;
        }
    }
}

static float _get_junction_vmax(const float a_unit[], const float b_unit[]) 
{
    float costheta = 0;
    for (int i = 0; i < AXES; i++) 
    {
        costheta -= a_unit[i] * b_unit[i];
    }
    
    if (costheta < -0.99f) return 10000000.0f; // прямая линия
    if (costheta > 0.99f) return 0.0f;         // разворот
    
    float a_delta = 0, b_delta = 0;
    for (int i = 0; i < AXES; i++) 
    {
        a_delta += square(a_unit[i] * axis_params[i].junction_dev);
        b_delta += square(b_unit[i] * axis_params[i].junction_dev);
    }
    
    float delta = (sqrtf(a_delta) + sqrtf(b_delta)) / 2.0f;
    float sintheta_over2 = sqrtf((1.0f - costheta) / 2.0f);
    float radius = delta * sintheta_over2 / (1.0f - sintheta_over2);
    float velocity = sqrtf(radius * 100.0f); // junction_acceleration = 100 mm/s²
    return velocity;
}

