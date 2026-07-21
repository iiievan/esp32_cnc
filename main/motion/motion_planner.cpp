#include "motion.h"
#include "motion_timer.h"

extern "C" 
{
    void motion_timer_init(void);
    void start_motion_timer(void);
    void stop_motion_timer(void);
}
static const char *TAG = "MOT_PLAN";

// Параметры оси (пример для X и Y)
axis_param_t axis_params[AXES] = 
{
    AXIS_PARAM_DEFAULT(), // X
    AXIS_PARAM_DEFAULT()  // Y
};

// Глобальные переменные
mpMoveMaster_t      mm = MPMOVE_MASTER_INIT();      // фактическая позиция и параметры движения
mpBuf_t             bf = MPBUF_INIT();              // планируемое движение 
mpMoveRuntime_t     mr = MPMOVE_RUNTIME_INIT();     // исполнение запланированного движения 
SemaphoreHandle_t bf_mutex = NULL; // мьютекс для защиты буфера.
SemaphoreHandle_t motion_complete_sem = NULL;   // семафор для обозначения окончания движения

// Предыдущий unit-вектор для расчета junction_velocity
static MATH::Vector2D<float>  prev_unit;

static void _calc_move_times(mpBuf_t *bf, const MATH::Vector2D<float>& delta);
static float _get_junction_vmax(const MATH::Vector2D<float>& a_unit, const MATH::Vector2D<float>& b_unit);

void mp_init(void) 
{
    memset(&mm, 0, sizeof(mm));
    memset(&mr, 0, sizeof(mr));
    memset(&bf, 0, sizeof(bf));
    prev_unit = MATH::Vector2D<float>::zero();

    // reset start position to zero
    mm.position.zero();
    
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
    float cx = (current_x != NULL) ? *current_x : mm.position.x;
    float cy = (current_y != NULL) ? *current_y : mm.position.y;
    
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
    using namespace MATH;

    ESP_LOGI(TAG, "=== mp_aline() START: target=(%.2f, %.2f), feed=%.2f ===", 
             target_x, target_y, feed_rate);
    ESP_LOGI(TAG, "  mm.position = (%.2f, %.2f)", mm.position.x, mm.position.y);

    if (mr.block_state == BLOCK_RUNNING) 
    {
        ESP_LOGW(TAG, "Planner busy, cannot start new move");
        return false;
    }

    if (is_zero_move_default(target_x, target_y)) 
    {
        ESP_LOGW(TAG, "Zero length move to (%.2f, %.2f) - skipping", target_x, target_y);

        mm.position = Vector2D<float>(target_x,target_y);

        return true; // Возвращаем true, так как уже в этой позиции
    }

    if (xSemaphoreTake(bf_mutex, portMAX_DELAY) == pdTRUE) 
    {
        // Очищаем буфер
        memset(&bf, 0, sizeof(mpBuf_t));
        bf.block_state = BLOCK_RUNNING; 
    
        // Вычисляем длины по осям
        Vector2D<float> axis_length = Vector2D<float>(target_x, target_y) - mm.position;

        bf.length = axis_length.length();

        ESP_LOGI(TAG, "  bf.length = %.4f", bf.length);

        if (fp_ZERO(bf.length)) 
        {
            ESP_LOGW(TAG, "Zero length move");
            return false;
        }

        // Устанавливаем feed rate
        bf.gm.feed_rate = feed_rate; // мм/мин
        
        // Вычисляем время движения
        _calc_move_times(&bf, axis_length);

        ESP_LOGI(TAG, "  bf.gm.move_time = %.6f min", bf.gm.move_time); 
        
        bf.unit = axis_length.normalized();

        // Сравниваем вклад X и Y в ограничение Jerk
        float cx = bf.unit.x * bf.unit.x * axis_params[0].recip_jerk;
        float cy = bf.unit.y * bf.unit.y * axis_params[1].recip_jerk;

        // Нам важен только индекс лимитирующей оси (0 - X, 1 - Y)
        bf.jerk_axis = (cx > cy) ? 0 : 1;

        // 1. Вычисляем лимитирующий jerk по выбранной оси (0 -> X, 1 -> Y)
        float unit_component = (bf.jerk_axis == 0) ? bf.unit.x : bf.unit.y;
        
        // Защита от деления на 0 при близких к 0 значениях компоненты
        float abs_unit_comp = std::abs(unit_component);
        if (abs_unit_comp < 1e-6f) abs_unit_comp = 1e-6f;

        bf.jerk = (axis_params[bf.jerk_axis].jerk_max * JERK_MULTIPLIER) / abs_unit_comp;
        bf.recip_jerk = 1.0f / bf.jerk;
        bf.cbrt_jerk = std::cbrt(bf.jerk);

        ESP_LOGI(TAG, "  bf.jerk = %.2f, axis=%d", bf.jerk, bf.jerk_axis);

        // 2. Устанавливаем скорости
        bf.cruise_vmax = bf.length / bf.gm.move_time; 

        // 3. Junction velocity с учетом предыдущего движения (передача по ссылке)
        float junction_velocity = _get_junction_vmax(prev_unit, bf.unit);
        
        // 4. Прямое присваивание вектора вместо copy_vector!
        prev_unit = bf.unit;

        // 5. Флаги и остановка
        constexpr float exact_stop = 8675309.0f;
        bf.replannable = true;

        bf.entry_vmax = std::min(std::min(bf.cruise_vmax, junction_velocity), exact_stop);
        bf.delta_vmax = mp_get_target_velocity(0.0f, bf.length, &bf);
        bf.exit_vmax = std::min(std::min(bf.cruise_vmax, (bf.entry_vmax + bf.delta_vmax)), exact_stop);
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

        mr.unit = bf.unit;
        mr.target = mm.position + axis_length;
        mr.position = mm.position;

        mr.head_length = bf.head_length;
        mr.body_length = bf.body_length;
        mr.tail_length = bf.tail_length;

        mr.entry_velocity = bf.entry_velocity;
        mr.cruise_velocity = bf.cruise_velocity;
        mr.exit_velocity = bf.exit_velocity;

        mm.position = bf.gm.target; 

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

static void _calc_move_times(mpBuf_t *bf, const MATH::Vector2D<float>& delta) 
{
    float xyz_time = 0.0f;
    
    bf->gm.minimum_time = 8675309.0f; // Большое число по умолчанию
    
    float length = bf->length;
    if (length > 0.0f && bf->gm.feed_rate > 0.0f) 
    {
        xyz_time = length / bf->gm.feed_rate; // Время в минутах (или секундах, в зависимости от единиц feed_rate)
    }

    // Время по координатам X и Y через компоненты вектора
    float time_x = (std::abs(delta.x) > 0.0f) ? (std::abs(delta.x) / bf->gm.feed_rate) : 0.0f;
    float time_y = (std::abs(delta.y) > 0.0f) ? (std::abs(delta.y) / bf->gm.feed_rate) : 0.0f;

    float max_time = std::max(time_x, time_y);

    // Расчет minimum_time только для активных осей
    if (time_x > 0.0f) bf->gm.minimum_time = std::min(bf->gm.minimum_time, time_x);
    if (time_y > 0.0f) bf->gm.minimum_time = std::min(bf->gm.minimum_time, time_y);

    bf->gm.move_time = std::max(xyz_time, max_time);
    
    // Проверка на короткие движения (S-curve jerk limits)
    if (bf->gm.move_time < MIN_BLOCK_TIME) 
    {
        float delta_velocity = std::pow(length, 0.66666666f) * mm.cbrt_jerk;
        float entry_velocity = 0.0f; // Для первого движения entry_velocity = 0
        
        float move_time = (2.0f * length) / (2.0f * entry_velocity + delta_velocity);
        
        bf->gm.move_time = (move_time < MIN_BLOCK_TIME) ? MIN_BLOCK_TIME : move_time;
    }
}

static float _get_junction_vmax(const MATH::Vector2D<float>& a_unit, const MATH::Vector2D<float>& b_unit) 
{
    // В GRBL косинус угла между входящим и исходящим вектором движения:
    // cos(theta) = - (a_unit • b_unit)
    float costheta = -a_unit.dot(b_unit);

    // Пограничные случаи:
    if (costheta < -0.99f) return 10000000.0f; // Прямая линия (почти 180 градусов между направлениями)
    if (costheta > 0.99f)  return 0.0f;        // Острый разворот на 180 градусов назад

    // Вычисляем отклонение (junction deviation) 
    float a_dev_x = a_unit.x * axis_params[0].junction_dev;
    float a_dev_y = a_unit.y * axis_params[1].junction_dev;
    
    float b_dev_x = b_unit.x * axis_params[0].junction_dev;
    float b_dev_y = b_unit.y * axis_params[1].junction_dev;

    float a_delta = a_dev_x * a_dev_x + a_dev_y * a_dev_y;
    float b_delta = b_dev_x * b_dev_x + b_dev_y * b_dev_y;

    float delta = (std::sqrt(a_delta) + std::sqrt(b_delta)) * 0.5f;

    // Тригонометрия для радиуса скругления на стыке (Junction Radius):
    float sintheta_over2 = std::sqrt((1.0f - costheta) * 0.5f);
    
    // Защита от деления на ноль:
    if (std::abs(1.0f - sintheta_over2) < 1e-6f) return 0.0f;

    float radius = (delta * sintheta_over2) / (1.0f - sintheta_over2);
    
    constexpr float junction_acceleration = 100.0f; // mm/s²
    return std::sqrt(radius * junction_acceleration);
}

