#include "motion.h"
#include "esp_log.h"

static const char *TAG = "MOT_TRACK";

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

const char* block_state_to_str(blockState_t state) 
{
    switch(state) 
    {
        case BLOCK_IDLE: return "IDLE";
        case BLOCK_INITIALIZING: return "INITIALIZING";
        case BLOCK_RUNNING: return "RUNNING";
        default: return "UNKNOWN";
    }
}

const char* section_to_str(moveSection_t section) 
{
    switch(section) 
    {
        case SECTION_NA: return "N/A";
        case SECTION_HEAD: return "HEAD";
        case SECTION_BODY: return "BODY";
        case SECTION_TAIL: return "TAIL";
        default: return "UNKNOWN";
    }
}

const char* section_state_to_str(sectionState_t state) 
{
    switch(state) 
    {
        case SECTION_OFF: return "OFF";
        case SECTION_NEW: return "NEW";
        case SECTION_1st_HALF: return "1st_HALF";
        case SECTION_2nd_HALF: return "2nd_HALF";
        default: return "UNKNOWN";
    }
}

void track_motion_states()
{
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
}