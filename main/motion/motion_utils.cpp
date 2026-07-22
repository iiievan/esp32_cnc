#include "motion_planner.hpp"

static const char *TAG = "MOT_TRACK";

const char* MotionPlanner::block_state_to_str(blockState_t state) const noexcept
{
    switch(state) 
    {
        case BLOCK_IDLE: return "IDLE";
        case BLOCK_INITIALIZING: return "INITIALIZING";
        case BLOCK_RUNNING: return "RUNNING";
        default: return "UNKNOWN";
    }
}

const char* MotionPlanner::section_to_str(moveSection_t section) const noexcept
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

const char* MotionPlanner::section_state_to_str(sectionState_t state) const noexcept
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

void MotionPlanner::track_motion_states() noexcept
{
    if (_mr.block_state != old_block_state) 
    {
        ESP_LOGI(TAG, "Block state: %s -> %s", 
                 block_state_to_str(old_block_state), 
                 block_state_to_str(_mr.block_state));
        old_block_state = _mr.block_state;
    }

    if (_mr.section != old_section) 
    {
        ESP_LOGI(TAG, "Section: %s -> %s", 
                 section_to_str(old_section), 
                 section_to_str(_mr.section));
        old_section = _mr.section;
    }

    if (_mr.section_state != old_section_state) 
    {
        ESP_LOGI(TAG, "Section state: %s -> %s", 
                 section_state_to_str(old_section_state), 
                 section_state_to_str(_mr.section_state));
        old_section_state = _mr.section_state;
    }
}