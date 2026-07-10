#ifndef GRBL_PARSER_H
#define GRBL_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum 
{
    CMD_UNKNOWN = 0,
    CMD_MOTION_LINEAR,        // G0, G1
    CMD_MOTION_ARC,           // G2, G3
    CMD_STOP,                 // M5
    CMD_HOME,                 // G28
    CMD_SPEED,                // F (скорость)
    CMD_DWELL,                // G4 (пауза)
    CMD_ABSOLUTE,             // G90
    CMD_RELATIVE,             // G91
    CMD_UNITS_MM,             // G21
    CMD_UNITS_INCH            // G20
} grbl_cmd_type_t;


typedef struct 
{
    grbl_cmd_type_t type;
    float x;          // Координата X
    float y;          // Координата Y
    float z;          // Координата Z
    float f;          // Скорость (Feed rate)
    float i;          // Для дуг (центр X)
    float j;          // Для дуг (центр Y)
    float r;          // Для дуг (радиус)
    float p;          // Для паузы (время)
    bool has_x;
    bool has_y;
    bool has_z;
    bool has_f;
} parsed_command_t;

bool grbl_parse(const char *cmd, parsed_command_t *result);
const char* grbl_cmd_type_to_string(grbl_cmd_type_t type);

#ifdef __cplusplus
}
#endif

#endif // GRBL_PARSER_H