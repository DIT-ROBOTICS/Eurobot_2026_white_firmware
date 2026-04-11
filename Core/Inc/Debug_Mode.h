/*
 * Debug_Mode.h
 *
 *  Created on: Apr 8, 2026
 *      Author: jason
 */

#ifndef INC_DEBUG_MODE_H_
#define INC_DEBUG_MODE_H_
#ifdef __cplusplus
extern "C"{
#endif

void mission_debugger(void);

extern int *is_in_debugger_mode;

#ifdef __cplusplus

}
#include "type_define.h"

struct Debug_Config
{
    int is_in_debugger_mode;
    int now_process;
    int last_process;
    SideIndex Mission_Side;
    int *is_for_ninja;
    bool should_random;
    MissionCaller Mission_Caller;
    int *task_execution_status;
    struct ServoTest{ 
        ServoConfig_Name name; 
        int exec_time; 
        bool had_exec; 
    } servo_test;
};
#endif 

#endif /* INC_DEBUG_MODE_H_ */
