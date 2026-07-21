#ifndef _MOTION_PLANNER_H
#define _MOTION_PLANNER_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DBG_PLANNER_LOG

#define AXES                    (2)             // X, Y
#define MOTORS                  (2)             // X, Y
#define MICROSECONDS_PER_MINUTE (60000000.0f)
#define MIN_SEGMENT_USEC        (2500)
#define MIN_SEGMENT_TIME        (MIN_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define MIN_BLOCK_TIME          (MIN_SEGMENT_TIME)

#define JERK_MULTIPLIER         (1.0f)
#define JERK_MATCH_PRECISION    (1000.0f)
#define NOM_SEGMENT_USEC        (5000)
#define NOM_SEGMENT_TIME        (NOM_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)

#define square(x)               ((x)*(x))
#define max(a,b)                (((a)>(b))?(a):(b))
#define min(a,b)                (((a)<(b))?(a):(b))
#define max3(a,b,c)             max(max(a,b),c)
#define min3(a,b,c)             min(min(a,b),c)
#define max4(a,b,c,d)           max(max(a,b),max(c,d))
#define min4(a,b,c,d)           min(min(a,b),min(c,d))
#define EPSILON		            ((float)0.00001)		// allowable rounding error for floats
#define fp_ZERO(x)              (fabs(x) < EPSILON)
#define fp_EQ(a,b)              (fabs((a)-(b)) < EPSILON)
#define	copy_vector(d,s)        (memcpy(d,s,sizeof(d)))

typedef enum 
{
    STAT_OK = 0,
    STAT_EAGAIN = 1,
    STAT_NOOP = 2,
    STAT_MINIMUM_TIME_MOVE = 3,
    STAT_INTERNAL_ERROR = 4
} stat_t;

typedef enum 
{
    BLOCK_IDLE = 0,
    BLOCK_INITIALIZING,
    BLOCK_RUNNING
} blockState_t;

typedef enum 
{
    SECTION_NA = -1,
    SECTION_HEAD = 0,
    SECTION_BODY,
    SECTION_TAIL
} moveSection_t;
#define SECTIONS (3)

typedef enum 
{
    SECTION_OFF = 0,
    SECTION_NEW,
    SECTION_1st_HALF,
    SECTION_2nd_HALF
} sectionState_t;

typedef struct 
{
    float jerk_max;
    float recip_jerk;
    float junction_dev;
} axis_param_t;

#define AXIS_PARAM_DEFAULT() { \
    .jerk_max = 25.0f, \
    .recip_jerk = 1.0f, \
    .junction_dev = 0.01f \
}

// Gcode model state - used by model, planning and runtime
typedef struct GCodeState 
{											
	float target[AXES]; 				// XYZABC where the move should go
	float move_time;					// optimal time for move given axis constraints
	float minimum_time;					// minimum time possible for move given axis constraints
	float feed_rate; 					// F - normalized to millimeters/minute or in inverse time mode
} GCodeState_t;

// Планировщик буфер (у нас будет только один)
typedef struct mpBuf 
{
    blockState_t block_state;

    float unit[AXES];
    float length;
    float head_length;
    float body_length;
    float tail_length;
    uint8_t replannable;
    
    float entry_velocity;
    float cruise_velocity;
    float exit_velocity;
    
    float entry_vmax;
    float cruise_vmax;
    float exit_vmax;
    float delta_vmax;
    float braking_velocity;
  
    uint8_t jerk_axis; 
    float jerk;
    float recip_jerk;
    float cbrt_jerk;

    GCodeState_t gm;				// Gode model state - passed from model, used by planner and runtime
} mpBuf_t;

#define MPBUF_INIT() { \
    .block_state = BLOCK_IDLE, \
    .unit = {0.0f, 0.0f}, \
    .length = 0.0f, \
    .head_length = 0.0f, \
    .body_length = 0.0f, \
    .tail_length = 0.0f, \
    .replannable = 0, \
    .entry_velocity = 0.0f, \
    .cruise_velocity = 0.0f, \
    .exit_velocity = 0.0f, \
    .entry_vmax = 0.0f, \
    .cruise_vmax = 0.0f, \
    .exit_vmax = 0.0f, \
    .delta_vmax = 0.0f, \
    .braking_velocity = 0.0f, \
    .jerk_axis = 0, \
    .jerk = 0.0f, \
    .recip_jerk = 0.0f, \
    .cbrt_jerk = 0.0f, \
    .gm = {} \
}

// Глобальные структуры (mm - master, mr - runtime)
typedef struct 
{
    float position[AXES];      // текущая позиция в мм
    float jerk;
    float recip_jerk;
    float cbrt_jerk;
} mpMoveMaster_t;

#define MPMOVE_MASTER_INIT() { \
    .position = {0.0f, 0.0f}, \
    .jerk = 20.0f, \
    .recip_jerk = 0.05f, \
    .cbrt_jerk = 2.714f \
}

typedef struct 
{
    blockState_t block_state;
    moveSection_t section;
    sectionState_t section_state;
    
    float unit[AXES];
    float target[AXES];
    float position[AXES];
    float waypoint[SECTIONS][AXES];   // head, body, tail endpoints   
    
    float head_length;
    float body_length;
    float tail_length;
    
    float entry_velocity;
    float cruise_velocity;
    float exit_velocity;
    
    float segments;
    uint32_t segment_count;
    float segment_velocity;
    float segment_time;
    
    float forward_diff_1;
    float forward_diff_2;
    float forward_diff_3;
    float forward_diff_4;
    float forward_diff_5;

    GCodeState_t gm;				// gcode model state currently executing
} mpMoveRuntime_t;

#define MPMOVE_RUNTIME_INIT() { \
    .block_state = BLOCK_IDLE, \
    .section = SECTION_HEAD, \
    .section_state = SECTION_OFF, \
    .unit = {0.0f, 0.0f}, \
    .target = {0.0f, 0.0f}, \
    .position = {0.0f, 0.0f}, \
    .waypoint = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}}, \
    .head_length = 0.0f, \
    .body_length = 0.0f, \
    .tail_length = 0.0f, \
    .entry_velocity = 0.0f, \
    .cruise_velocity = 0.0f, \
    .exit_velocity = 0.0f, \
    .segments = 0.0f, \
    .segment_count = 0, \
    .segment_velocity = 0.0f, \
    .segment_time = 0.0f, \
    .forward_diff_1 = 0.0f, \
    .forward_diff_2 = 0.0f, \
    .forward_diff_3 = 0.0f, \
    .forward_diff_4 = 0.0f, \
    .forward_diff_5 = 0.0f, \
    .gm = {} \
}

extern mpMoveMaster_t      mm;
extern mpMoveRuntime_t     mr;
extern mpBuf_t             bf; 
extern SemaphoreHandle_t bf_mutex;
extern SemaphoreHandle_t motion_complete_sem;

void mp_init(void);
bool mp_aline(float target_x, float target_y, float feed_rate);
stat_t mp_exec_aline(mpBuf_t *bf);

static inline float uSec(float minutes) { return minutes * MICROSECONDS_PER_MINUTE; }
float mp_get_target_velocity(const float Vi, const float L, const mpBuf_t *bf);
float mp_get_target_length(const float Vi, const float Vf, const mpBuf_t *bf);
const char* block_state_to_str(blockState_t state); 
const char* section_to_str(moveSection_t section); 
const char* section_state_to_str(sectionState_t state);

/**
 * @brief Проверяет, является ли перемещение нулевым
 * @param target_x Целевая X координата
 * @param target_y Целевая Y координата
 * @param current_x Текущая X координата (опционально, если NULL - использует mm.position)
 * @param current_y Текущая Y координата (опционально, если NULL - использует mm.position)
 * @param tolerance Допуск в мм (по умолчанию 0.001)
 * @return true если перемещение нулевое (в пределах допуска)
 */
bool is_zero_move(float target_x, float target_y, const float* current_x, const float* current_y);

static inline bool is_zero_move_default(float target_x, float target_y) 
{
    return is_zero_move(target_x, target_y, NULL, NULL);
}

#ifdef __cplusplus
}
#endif

#endif  //_MOTION_PLANNER_H