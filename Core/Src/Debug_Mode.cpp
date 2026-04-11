/*
 * Debug_Mode.cpp
 *
 *  Created on: Apr 8, 2026
 *      Author: jason
 */

#include "Debug_Mode.h"
#include "FreeRTOS.h"
#include "mission.h"
#include "Servo_Config.h"
#include <cstdio>
#include <stdlib.h>



Debug_Config debug_Config = {
    false,          // is_in_debugger_mode
    0,              // now_process
    0,              // last_process
    SIDE_B,         // Mission_Side
    &is_for_ninja,  // int*
    false,          // should_random
    {},             // Mission_Caller
    &process,       // int*
    {               // servo_test
        takeup_up,
        0,
        false
    }
};

int *is_in_debugger_mode = &debug_Config.is_in_debugger_mode;

// HeapStats_t HeapStats;


void mission_debugger(void){
    // vPortGetHeapStats(&HeapStats);
    if(!debug_Config.servo_test.had_exec){
        arms[debug_Config.Mission_Side] -> do_async(debug_Config.servo_test.name, debug_Config.servo_test.exec_time);
        debug_Config.servo_test.had_exec = true;
    }
    if(debug_Config.now_process == debug_Config.last_process)return;
    debug_Config.last_process = debug_Config.now_process;
    if(debug_Config.now_process == 1)debug_Config.Mission_Caller.mission_type = TakeUpBlock;
    else if(debug_Config.now_process == 2)debug_Config.Mission_Caller.mission_type = FlipBlock;
    else if(debug_Config.now_process == 3)debug_Config.Mission_Caller.mission_type = PutDownBlock;
    else if(debug_Config.now_process == 4)debug_Config.Mission_Caller.mission_type = CloseMission;
    else if(debug_Config.now_process == 5)debug_Config.Mission_Caller.mission_type = AlignBlock;
    else if(debug_Config.now_process == 6)debug_Config.Mission_Caller.mission_type = PushTemperture;
    else if(debug_Config.now_process == 7)return;//push_temp(debug_Config.Mission_Caller.push_temp_side,debug_Config.Mission_Caller.push_temp);
    else return;
    if(debug_Config.now_process == 1 && debug_Config.should_random){
        srand(HAL_GetTick());
        uint32_t r = rand();
        debug_Config.Mission_Caller.pinecone[0] = (r & 1) != 0;
        debug_Config.Mission_Caller.pinecone[1] = (r & 2) != 0;
        debug_Config.Mission_Caller.pinecone[2] = (r & 4) != 0;
        debug_Config.Mission_Caller.pinecone[3] = (r & 8) != 0;
    }
    if(debug_Config.Mission_Side < SUBMISSION || debug_Config.Mission_Side > SIDE_L)return;
    mission_deliveryman((SideIndex)debug_Config.Mission_Side,&debug_Config.Mission_Caller);
}



