// main/grbl_parser.c
#include "grbl_parser.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

bool grbl_parse(const char *cmd, parsed_command_t *result) 
{
    if (cmd == NULL || result == NULL) {
        return false;
    }
    
    memset(result, 0, sizeof(parsed_command_t));   

    char cmd_copy[64];
    strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';
    

    char *p = cmd_copy;
    while (*p && (*p == ' ' || *p == '\n' || *p == '\r')) p++;
    
    if (*p == '\0') 
    {
        return false;
    }
    
    // Определяем тип команды по первым символам
    if (strncmp(p, "G1", 2) == 0 || strncmp(p, "G0", 2) == 0) 
    {
        result->type = CMD_MOTION_LINEAR;
    } 
    else if (strncmp(p, "G2", 2) == 0 || strncmp(p, "G3", 2) == 0) 
    {
        result->type = CMD_MOTION_ARC;
    } 
    else if (strncmp(p, "G28", 3) == 0) 
    {
        result->type = CMD_HOME;
    } 
    else if (strncmp(p, "M5", 2) == 0) 
    {
        result->type = CMD_STOP;
    } 
    else if (strncmp(p, "G4", 2) == 0) 
    {
        result->type = CMD_DWELL;
    } 
    else if (strncmp(p, "G90", 3) == 0) 
    {
        result->type = CMD_ABSOLUTE;
    } 
    else if (strncmp(p, "G91", 3) == 0) 
    {
        result->type = CMD_RELATIVE;
    } 
    else if (strncmp(p, "G21", 3) == 0) 
    {
        result->type = CMD_UNITS_MM;
    } 
    else if (strncmp(p, "G20", 3) == 0) 
    {
        result->type = CMD_UNITS_INCH;
    } 
    else 
    {
        result->type = CMD_UNKNOWN;
        return false;
    }
    
    // Если это команда движения, парсим параметры
    if (result->type == CMD_MOTION_LINEAR || result->type == CMD_MOTION_ARC) 
    {
        char *token = strtok(p, " ");
        while (token != NULL) 
        {
            if (token[0] == 'X' || token[0] == 'x') 
            {
                result->x = strtof(token + 1, NULL);
                result->has_x = true;
            } 
            else if (token[0] == 'Y' || token[0] == 'y') 
            {
                result->y = strtof(token + 1, NULL);
                result->has_y = true;
            } 
            else if (token[0] == 'F' || token[0] == 'f') 
            {
                result->f = strtof(token + 1, NULL);
                result->has_f = true;
            } 
            else if (token[0] == 'I' || token[0] == 'i') 
            {
                result->i = strtof(token + 1, NULL);
            } 
            else if (token[0] == 'J' || token[0] == 'j') 
            {
                result->j = strtof(token + 1, NULL);
            } 
            else if (token[0] == 'R' || token[0] == 'r') 
            {
                result->r = strtof(token + 1, NULL);
            } 
            else if (token[0] == 'P' || token[0] == 'p') 
            {
                result->p = strtof(token + 1, NULL);
            }

            token = strtok(NULL, " ");
        }
    }
    
    return true;
}

const char* grbl_cmd_type_to_string(grbl_cmd_type_t type) 
{
    switch(type) 
    {
        case CMD_MOTION_LINEAR: 
            return "LINEAR";
        case CMD_MOTION_ARC: 
            return "ARC";
        case CMD_STOP: 
            return "STOP";
        case CMD_HOME: 
            return "HOME";
        case CMD_SPEED: 
            return "SPEED";
        case CMD_DWELL: 
            return "DWELL";
        case CMD_ABSOLUTE: 
            return "ABSOLUTE";
        case CMD_RELATIVE: 
            return "RELATIVE";
        case CMD_UNITS_MM: 
            return "UNITS_MM";
        case CMD_UNITS_INCH: 
            return "UNITS_INCH";
        default: 
            return "UNKNOWN";
    }
}