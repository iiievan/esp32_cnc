#include "motion_planner.hpp"

static const char *TAG = "MOT_PLAN";
static const char *TAGE = "MOT_EXECUTE";

MotionPlanner* G_plnr = nullptr;

void MotionPlanner::log_promt()
{
    ESP_LOGI(TAGE, "MOVE START");
    ESP_LOGI(TAGE, "pos_x_mm:pos_y_mm:vel_mm_min:T_ms:dt_ms");
}

void MotionPlanner::log_out(int T_ms, int dt_ms)
{
    ESP_LOGI(TAGE, "%.3f:%.3f:%.2f:%d:%d", _mr.position.x, _mr.position.y, 
                                          _mr.segment_velocity * 60.0f, 
                                          T_ms, 
                                          dt_ms);
}

stat_t MotionPlanner::exec_segment(void) 
{
    _mr.segment_count--;
    
    // Если это последний сегмент и мы в конце секции — используем waypoint
    if (_mr.segment_count == 0 && _mr.section_state == SECTION_2nd_HALF) 
    {
        _mr.gm.target = _mr.waypoint[_mr.section];
    } 
    else 
    {
        float segment_length = _mr.segment_velocity * _mr.segment_time;
		_mr.gm.target = _mr.position + (_mr.unit * segment_length);
    }

#ifdef DBG_EXE_LOG
    static moveSection_t old_section = SECTION_NA;
    if (_mr.section != old_section) 
    {
        ESP_LOGI(TAGE, "Pos old:%.3f, %.3f", _mr.position.x, _mr.position.y);
        old_section = _mr.section;
    }
#endif    
    
    _mr.position = _mr.gm.target; // Обновляем позицию из _mr.gm.target для нового сегмента

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

    if (_mr.segment_count == 0) return (STAT_OK);    
    return STAT_EAGAIN;
}

MotionPlanner::MotionPlanner(void)
{
    _mm = MPMOVE_MASTER_INIT();      
    _bf = MPBUF_INIT();              
    _mr = MPMOVE_RUNTIME_INIT();      
    _prev_unit = MATH::Vector2D<float>::zero();
    _mm.position.zero();
} 

MotionPlanner::~MotionPlanner(void)
{
    if (_motion_timer != nullptr) 
    {
        stop_timer();
        esp_timer_delete(_motion_timer);
        _motion_timer = nullptr;
    }

    if (_bf_mutex != NULL) 
    {
        vSemaphoreDelete(_bf_mutex);
        _bf_mutex = NULL;
    }

    if (_motion_complete_sem == NULL) 
    {
        vSemaphoreDelete(_motion_complete_sem);
        _motion_complete_sem = NULL;
    }
}

bool MotionPlanner::init(void) 
{    
    for (int i = 0; i < AXES; i++) 
    {
        _axes[i] = Axis();
    }

    if (_bf_mutex == nullptr) 
    {
        _bf_mutex = xSemaphoreCreateMutex();
        if (_bf_mutex == nullptr) 
        {
            ESP_LOGE(TAG, "Failed to create bf_mutex");
            return false;
        }
    }

    if (_motion_complete_sem == nullptr) 
    {
        _motion_complete_sem = xSemaphoreCreateBinary();
        if (_motion_complete_sem == nullptr) 
        {
            ESP_LOGE(TAG, "Failed to create _motion_complete_sem");
            return false;
        }
    }

    if (_motion_timer == nullptr) 
    {
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = &MotionPlanner::timer_callback;;
        timer_args.arg = this;
        timer_args.name = "motion_timer";

        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &_motion_timer));
        _timer_running = false;
        ESP_LOGI("TIMER", "Motion timer created successfully");
    }    
    
    ESP_LOGI(TAG, "Motion planner initialized");
    return true;
}

bool MotionPlanner::plan_linear_move(float target_x, float target_y, float feed_rate) 
{
    using namespace MATH;

    ESP_LOGI(TAG, "=== mp_aline() START: target=(%.2f, %.2f), feed=%.2f ===", 
             target_x, target_y, feed_rate);
    ESP_LOGI(TAG, "  _mm.position = (%.2f, %.2f)", _mm.position.x, _mm.position.y);

    if (_mr.block_state == BLOCK_RUNNING) 
    {
        ESP_LOGW(TAG, "Planner busy, cannot start new move");
        return false;
    }

    if (is_zero_move(target_x, target_y, NULL, NULL)) 
    {
        ESP_LOGW(TAG, "Zero length move to (%.2f, %.2f) - skipping", target_x, target_y);

        _mm.position = Vector2D<float>(target_x,target_y);

        return true; // Возвращаем true, так как уже в этой позиции
    }

    if (xSemaphoreTake(_bf_mutex, portMAX_DELAY) == pdTRUE) 
    {
        // Очищаем буфер
        memset(&_bf, 0, sizeof(mpBuf_t));
        _bf.block_state = BLOCK_RUNNING; 
    
        // Вычисляем длины по осям
        Vector2D<float> axis_length = Vector2D<float>(target_x, target_y) - _mm.position;

        _bf.length = axis_length.length();

        ESP_LOGI(TAG, "  _bf.length = %.4f", _bf.length);

        if (fp_ZERO(_bf.length)) 
        {
            ESP_LOGW(TAG, "Zero length move");
            return false;
        }

        // Устанавливаем feed rate
        _bf.gm.feed_rate = feed_rate; // мм/мин
        
        // Вычисляем время движения
        calculate_move_times(axis_length);

        ESP_LOGI(TAG, "  _bf.gm.move_time = %.6f min", _bf.gm.move_time); 
        
        _bf.unit = axis_length.normalized();

        // Сравниваем вклад X и Y в ограничение Jerk
        float cx = _bf.unit.x * _bf.unit.x * _axes[0].recip_jerk;
        float cy = _bf.unit.y * _bf.unit.y * _axes[1].recip_jerk;

        // Нам важен только индекс лимитирующей оси (0 - X, 1 - Y)
        _bf.jerk_axis = (cx > cy) ? 0 : 1;

        // 1. Вычисляем лимитирующий jerk по выбранной оси (0 -> X, 1 -> Y)
        float unit_component = (_bf.jerk_axis == 0) ? _bf.unit.x : _bf.unit.y;
        
        // Защита от деления на 0 при близких к 0 значениях компоненты
        float abs_unit_comp = std::abs(unit_component);
        if (abs_unit_comp < 1e-6f) abs_unit_comp = 1e-6f;

        _bf.jerk = (_axes[_bf.jerk_axis].jerk_max * JERK_MULTIPLIER) / abs_unit_comp;
        _bf.recip_jerk = 1.0f / _bf.jerk;
        _bf.cbrt_jerk = std::cbrt(_bf.jerk);

        ESP_LOGI(TAG, "  _bf.jerk = %.2f, axis=%d", _bf.jerk, _bf.jerk_axis);

        // 2. Устанавливаем скорости
        _bf.cruise_vmax = _bf.length / _bf.gm.move_time; 

        // 3. Junction velocity с учетом предыдущего движения (передача по ссылке)
        float junction_velocity = get_junction_vmax(_prev_unit, _bf.unit);
        
        // 4. Прямое присваивание вектора вместо copy_vector!
        _prev_unit = _bf.unit;

        // 5. Флаги и остановка
        constexpr float exact_stop = 8675309.0f;
        _bf.replannable = true;

        _bf.entry_vmax = std::min(std::min(_bf.cruise_vmax, junction_velocity), exact_stop);
        _bf.delta_vmax = get_target_velocity(0.0f, _bf.length);
        _bf.exit_vmax = std::min(std::min(_bf.cruise_vmax, (_bf.entry_vmax + _bf.delta_vmax)), exact_stop);
        _bf.braking_velocity = _bf.delta_vmax;
        _bf.entry_velocity = _bf.entry_vmax;
        _bf.cruise_velocity = _bf.cruise_vmax;
        _bf.exit_velocity = 0.0f;

        ESP_LOGI(TAG, "  speeds: entry=%.2f, cruise=%.2f, exit=%.2f", 
                 _bf.entry_velocity, _bf.cruise_velocity, _bf.exit_velocity);

        float total_length = _bf.length;
        float v_entry = _bf.entry_velocity;
        float v_cruise = _bf.cruise_velocity;
        float v_exit = _bf.exit_velocity;

        _bf.head_length = get_target_length(v_entry, v_cruise);
        _bf.tail_length = get_target_length(v_cruise, v_exit);
        if (_bf.tail_length < 0) _bf.tail_length = -_bf.tail_length;
        _bf.body_length = total_length - _bf.head_length - _bf.tail_length;

        ESP_LOGI(TAG, "  lengths: head=%.4f, body=%.4f, tail=%.4f", 
                 _bf.head_length, _bf.body_length, _bf.tail_length);

        // Если сумма head + tail больше общей длины — пропорционально уменьшаем
        if (_bf.body_length < 0) 
        {
            float total_accel = _bf.head_length + _bf.tail_length;
            if (total_accel > 0) 
            {
                _bf.head_length = (_bf.head_length / total_accel) * total_length;
                _bf.tail_length = (_bf.tail_length / total_accel) * total_length;
                _bf.body_length = 0.0f;
            }
            ESP_LOGI(TAG, "  corrected: head=%.4f, body=%.4f, tail=%.4f", 
                     _bf.head_length, _bf.body_length, _bf.tail_length);
        }

        float total_move_time = _bf.gm.move_time * 60.0f;
        _mr.segments = ceilf(total_move_time / (NOM_SEGMENT_USEC / 1000000.0f));
        if (_mr.segments < 10) _mr.segments = 10;    

        ESP_LOGI(TAG, "  _mr.segments = %.0f", _mr.segments);

        _mr.block_state = BLOCK_IDLE;
        _mr.section = SECTION_HEAD;
        _mr.section_state = SECTION_OFF;

        _mr.unit = _bf.unit;
        _mr.target = _mm.position + axis_length;
        _mr.position = _mm.position;

        _mr.head_length = _bf.head_length;
        _mr.body_length = _bf.body_length;
        _mr.tail_length = _bf.tail_length;

        _mr.entry_velocity = _bf.entry_velocity;
        _mr.cruise_velocity = _bf.cruise_velocity;
        _mr.exit_velocity = _bf.exit_velocity;

        _mm.position = _bf.gm.target; 

        ESP_LOGI(TAG, "Move planned: target=(%.2f, %.2f), length=%.2f, time=%.3f min, jerk=%.2f, segments=%.0f", 
                 target_x, target_y, _bf.length, _bf.gm.move_time, _bf.jerk, _mr.segments);

        xSemaphoreGive(_bf_mutex);   // отдаем мьютекс более высокопреоритетной задаче выполения движения stat_t mp_exec_aline(mpBuf_t *_bf) 
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
    start_timer();
    
    return true;
}

stat_t MotionPlanner::execute_move() 
{
    if (_bf.block_state == BLOCK_IDLE) 
    {
        return STAT_NOOP;
    }
    
    if (_mr.section_state == SECTION_OFF) 
    {
        log_promt();

        _bf.replannable = false;
        _bf.block_state = BLOCK_RUNNING;
        _mr.block_state = BLOCK_INITIALIZING;
        _mr.section = SECTION_HEAD;
        _mr.section_state = SECTION_NEW;
        _mm.jerk = _bf.jerk;
        
        _mr.head_length = _bf.head_length;
        _mr.body_length = _bf.body_length;
        _mr.tail_length = _bf.tail_length;
        
        _mr.entry_velocity = _bf.entry_velocity;
        _mr.cruise_velocity = _bf.cruise_velocity;
        _mr.exit_velocity = _bf.exit_velocity;

        _mr.unit = _bf.unit;
        _mr.target = _bf.gm.target;          
        
        // Вычисляем waypoints для коррекции позиции
        _mr.waypoint[SECTION_HEAD] = _mr.position + _mr.unit * _mr.head_length;
        _mr.waypoint[SECTION_BODY] = _mr.position + _mr.unit * (_mr.head_length + _mr.body_length);
        _mr.waypoint[SECTION_TAIL] = _mr.position + _mr.unit * (_mr.head_length + _mr.body_length + _mr.tail_length);

        
        _mr.block_state = BLOCK_RUNNING;
    }
    
    stat_t status = STAT_OK;
    switch(_mr.section)
    {
        case SECTION_HEAD:
            status = exec_head();
            break;

        case SECTION_BODY:
            status = exec_cruise();
            break;

        case SECTION_TAIL:
            status = exec_tail();
            break;
        default:
            ESP_LOGE(TAGE, "Invalid section");
            return STAT_INTERNAL_ERROR;
    }
    
    // Завершаем движение если нет новой секции
    if (status != STAT_EAGAIN) 
    {    
        _mr.section_state = SECTION_OFF;
        _mr.block_state = BLOCK_IDLE;
        _bf.block_state = BLOCK_IDLE;

        _mm.position = _mr.position;

        stop_timer();

        ESP_LOGI(TAGE, "MOVE COMPLETED AT (%.2f, %.2f)", _mr.position.x, _mr.position.y);
        
        if (_motion_complete_sem != nullptr) 
        {
            xSemaphoreGive(_motion_complete_sem);
        }
    }

    void track_motion_states();

    return status;
}

bool MotionPlanner::is_zero_move(float target_x, float target_y, const float* current_x, const float* current_y) const
{
    float cx = (current_x != NULL) ? *current_x : _mm.position.x;
    float cy = (current_y != NULL) ? *current_y : _mm.position.y;
    
    float dx = target_x - cx;
    float dy = target_y - cy;
    float distance = sqrtf(dx*dx + dy*dy);
    
    return (fabs(distance) < EPSILON);
}

void MotionPlanner::calculate_move_times(const MATH::Vector2D<float>& axis_length) 
{
    float xyz_time = 0.0f;
    
    _bf.gm.minimum_time = 8675309.0f; // Большое число по умолчанию
    
    float length = _bf.length;
    if (length > 0.0f && _bf.gm.feed_rate > 0.0f) 
    {
        xyz_time = length / _bf.gm.feed_rate; // Время в минутах (или секундах, в зависимости от единиц feed_rate)
    }

    // Время по координатам X и Y через компоненты вектора
    float time_x = (std::abs(axis_length.x) > 0.0f) ? (std::abs(axis_length.x) / _bf.gm.feed_rate) : 0.0f;
    float time_y = (std::abs(axis_length.y) > 0.0f) ? (std::abs(axis_length.y) / _bf.gm.feed_rate) : 0.0f;

    float max_time = std::max(time_x, time_y);

    // Расчет minimum_time только для активных осей
    if (time_x > 0.0f) _bf.gm.minimum_time = std::min(_bf.gm.minimum_time, time_x);
    if (time_y > 0.0f) _bf.gm.minimum_time = std::min(_bf.gm.minimum_time, time_y);

    _bf.gm.move_time = std::max(xyz_time, max_time);
    
    // Проверка на короткие движения (S-curve jerk limits)
    if (_bf.gm.move_time < MIN_BLOCK_TIME) 
    {
        float delta_velocity = std::pow(length, 0.66666666f) * _mm.cbrt_jerk;
        float entry_velocity = 0.0f; // Для первого движения entry_velocity = 0
        
        float move_time = (2.0f * length) / (2.0f * entry_velocity + delta_velocity);
        
        _bf.gm.move_time = (move_time < MIN_BLOCK_TIME) ? MIN_BLOCK_TIME : move_time;
    }
}

float MotionPlanner::get_junction_vmax(const MATH::Vector2D<float>& a_unit, const MATH::Vector2D<float>& b_unit) 
{
    // В GRBL косинус угла между входящим и исходящим вектором движения:
    // cos(theta) = - (a_unit • b_unit)
    float costheta = -a_unit.dot(b_unit);

    // Пограничные случаи:
    if (costheta < -0.99f) return 10000000.0f; // Прямая линия (почти 180 градусов между направлениями)
    if (costheta > 0.99f)  return 0.0f;        // Острый разворот на 180 градусов назад

    // Вычисляем отклонение (junction deviation) 
    float a_dev_x = a_unit.x * _axes[0].junction_dev;
    float a_dev_y = a_unit.y * _axes[1].junction_dev;
    
    float b_dev_x = b_unit.x * _axes[0].junction_dev;
    float b_dev_y = b_unit.y * _axes[1].junction_dev;

    float a_delta = a_dev_x * a_dev_x + a_dev_y * a_dev_y;
    float b_delta = b_dev_x * b_dev_x + b_dev_y * b_dev_y;

    float delta = (std::sqrt(a_delta) + std::sqrt(b_delta)) * 0.5f;

    // Тригонометрия для радиуса скругления на стыке (Junction Radius):
    float sintheta_over2 = std::sqrt((1.0f - costheta) * 0.5f);
    
    // Защита от деления на ноль:
    if (std::abs(1.0f - sintheta_over2) < 1e-6f) return 0.0f;

    float radius = (delta * sintheta_over2) / (1.0f - sintheta_over2);
    
    return std::sqrt(radius * JUNCTION_ACCELERATION);
}

void MotionPlanner::init_forward_diffs(float Vi, float Vt) 
{
    if (_mr.segments < 1) _mr.segments = 1;

    const float A = -6.0f  * Vi + 6.0f  * Vt;
    const float B =  15.0f * Vi - 15.0f * Vt;
    const float C = -10.0f * Vi + 10.0f * Vt;
    
    const float h = 1.0f / _mr.segments;
    const float Ah_5 = A * h * h * h * h * h;
    const float Bh_4 = B * h * h * h * h;
    const float Ch_3 = C * h * h * h;
    
    _mr.forward_diff_5 = (121.0f/16.0f)*Ah_5 + 5.0f*Bh_4 + (13.0f/4.0f)*Ch_3;
    _mr.forward_diff_4 = (165.0f/2.0f)*Ah_5 + 29.0f*Bh_4 + 9.0f*Ch_3;
    _mr.forward_diff_3 = 255.0f*Ah_5 + 48.0f*Bh_4 + 6.0f*Ch_3;
    _mr.forward_diff_2 = 300.0f*Ah_5 + 24.0f*Bh_4;
    _mr.forward_diff_1 = 120.0f*Ah_5;
    
    // Начальная скорость в середине первого сегмента
    float half_h = h / 2.0f;
    float half_Ch_3 = C * half_h * half_h * half_h;
    float half_Bh_4 = B * half_h * half_h * half_h * half_h;
    float half_Ah_5 = A * half_h * half_h * half_h * half_h * half_h;
    _mr.segment_velocity = half_Ah_5 + half_Bh_4 + half_Ch_3 + Vi;
 
#ifdef DBG_EXE_LOG
    ESP_LOGI(TAG, "  init_forward_diffs: Vi=%.2f, Vt=%.2f, segments=%.0f, h=%.6f", 
             Vi, Vt, _mr.segments, h);
    ESP_LOGI(TAG, "    forward_diff: d1=%.6f, d2=%.6f, d3=%.6f, d4=%.6f, d5=%.6f",
             _mr.forward_diff_1, _mr.forward_diff_2, _mr.forward_diff_3, 
             _mr.forward_diff_4, _mr.forward_diff_5);
    ESP_LOGI(TAG, "    segment_velocity = %.6f", _mr.segment_velocity);
#endif
}

stat_t MotionPlanner::exec_head(void) 
{
    if (_mr.section_state == SECTION_NEW) 
    {
        if (fp_ZERO(_mr.head_length)) 
        {
            _mr.section = SECTION_BODY;
            return exec_cruise();
        }
        
        const float move_time = 2.0f * _mr.head_length / (_mr.entry_velocity + _mr.cruise_velocity);
        
        // ВАЖНО: пересчитываем количество сегментов для этой секции
        float segments = ceil(get_uSec(move_time) / NOM_SEGMENT_USEC);
        if (segments < 1) segments = 1;
        _mr.segment_time = move_time / segments;
        
        // Используем локальное количество сегментов для forward differences
        _mr.segments = segments;
        _mr.segment_count = (uint32_t)segments;
        
        init_forward_diffs(_mr.entry_velocity, _mr.cruise_velocity);
        
        if (_mr.segment_time < MIN_SEGMENT_TIME) return STAT_MINIMUM_TIME_MOVE;
        
        _mr.section = SECTION_HEAD;
        _mr.section_state = SECTION_1st_HALF;
    }
    
    if (_mr.section_state == SECTION_1st_HALF) 
    {
        if (exec_segment() == STAT_OK) 
        {
            _mr.section = SECTION_BODY;
            _mr.section_state = SECTION_NEW;
            return STAT_EAGAIN; 
        } 
        else 
        {
            _mr.section_state = SECTION_2nd_HALF;
        }
        return STAT_EAGAIN;
    }
    
    if (_mr.section_state == SECTION_2nd_HALF) 
    {
        _mr.segment_velocity += _mr.forward_diff_5;

        if (exec_segment() == STAT_OK) 
        {
            if (fp_ZERO(_mr.body_length) && fp_ZERO(_mr.tail_length)) 
            {
                return STAT_OK;                    
            }
                
            _mr.section = SECTION_BODY;
            _mr.section_state = SECTION_NEW;
        } 
        else 
        {
            _mr.forward_diff_5 += _mr.forward_diff_4;
            _mr.forward_diff_4 += _mr.forward_diff_3;
            _mr.forward_diff_3 += _mr.forward_diff_2;
            _mr.forward_diff_2 += _mr.forward_diff_1;
        }
    }
    return STAT_EAGAIN;
}

stat_t MotionPlanner::exec_cruise(void) 
{
    if (_mr.section_state == SECTION_NEW) 
    {
        if (fp_ZERO(_mr.body_length)) 
        {
            _mr.section = SECTION_TAIL;
            return exec_tail();
        }
        
        const float move_time = _mr.body_length / _mr.cruise_velocity;
        float segments = ceil(get_uSec(move_time) / NOM_SEGMENT_USEC);
        if (segments < 1) segments = 1;
        _mr.segment_time = move_time / segments;
        _mr.segments = segments;
        _mr.segment_velocity = _mr.cruise_velocity;
        _mr.segment_count = (uint32_t)segments;
        
        if (_mr.segment_time < MIN_SEGMENT_TIME) return STAT_MINIMUM_TIME_MOVE;
        
        _mr.section = SECTION_BODY;
        _mr.section_state = SECTION_2nd_HALF;
    }
    
    if (_mr.section_state == SECTION_2nd_HALF) 
    {
        if (exec_segment() == STAT_OK) 
        {
            if (fp_ZERO(_mr.tail_length)) return STAT_OK;

            _mr.section = SECTION_TAIL;
            _mr.section_state = SECTION_NEW;
        }
    }
    return STAT_EAGAIN;
}

stat_t MotionPlanner::exec_tail(void) 
{
    if (_mr.section_state == SECTION_NEW) 
    {
        if (fp_ZERO(_mr.tail_length)) return STAT_OK;
        
        const float move_time = 2.0f * _mr.tail_length / (_mr.cruise_velocity + _mr.exit_velocity);
        float segments = ceil(get_uSec(move_time) / NOM_SEGMENT_USEC);
        if (segments < 1) segments = 1;
        _mr.segment_time = move_time / segments;
        _mr.segments = segments;
        _mr.segment_count = (uint32_t)segments;
        
        init_forward_diffs(_mr.cruise_velocity, _mr.exit_velocity);
        
        if (_mr.segment_time < MIN_SEGMENT_TIME) return STAT_MINIMUM_TIME_MOVE;
        
        _mr.section = SECTION_TAIL;
        _mr.section_state = SECTION_1st_HALF;
    }
    
    if (_mr.section_state == SECTION_1st_HALF) 
    {
        if (exec_segment() == STAT_OK) 
        {
            _mr.section_state = SECTION_2nd_HALF;
            return STAT_OK;
        } 
        else 
        {
            _mr.section_state = SECTION_2nd_HALF;
        }
        return STAT_EAGAIN;
    }
    
    if (_mr.section_state == SECTION_2nd_HALF) 
    {
        _mr.segment_velocity += _mr.forward_diff_5;

        if (exec_segment() == STAT_OK) 
        {
            return STAT_OK;
        } 
        else 
        {
            _mr.forward_diff_5 += _mr.forward_diff_4;
            _mr.forward_diff_4 += _mr.forward_diff_3;
            _mr.forward_diff_3 += _mr.forward_diff_2;
            _mr.forward_diff_2 += _mr.forward_diff_1;
        }
    }

    return STAT_EAGAIN;
}

void MotionPlanner::timer_callback(void* arg) 
{
    auto* planner = static_cast<MotionPlanner*>(arg);

    if (xSemaphoreTake(planner->_bf_mutex, 0) == pdTRUE && planner != nullptr) 
    {
        planner->execute_move(); 

        xSemaphoreGive(planner->_bf_mutex);
    }
    else
    {
#ifdef DBG_PLANNER_LOG
        ESP_LOGW("TIMER", "Database busy, skipping tick");
#endif       
    }
}

void MotionPlanner::start_timer() 
{
    if (_motion_timer != nullptr && !_timer_running) 
    {
        // Запуск периода 5 мс (5000 мкс)
        ESP_ERROR_CHECK(esp_timer_start_periodic(_motion_timer, 5000));
        _timer_running = true;
    }
}

void MotionPlanner::stop_timer() 
{
    if (_motion_timer != nullptr && _timer_running) 
    {
        ESP_ERROR_CHECK(esp_timer_stop(_motion_timer));
        _timer_running = false;
    }
}

