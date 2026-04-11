/*
 * subMission_Motion.cpp
 *
 *  Created on: Apr 8, 2026
 *      Author: jason
 */


#include "subMission_Motion.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "Servo_Config.h"


/* extern variable */
extern Servo Block_Align_R;
extern Servo Block_Align_L;
extern Servo Temperture_Align_R;
extern Servo Temperture_Align_L;
extern QueueHandle_t sideQueues[];
/* extern variable */

int sub_mission_sleeping = true;

void align_blocks(bool do_align);
void push_temp(SideIndex side,bool to_do);

void sub_servo_init(){
    if(!sub_mission_sleeping)return;
    Block_Align_R.setup(robot_config.r_align_angle_exit);
    Block_Align_L.setup(robot_config.l_align_angle_exit);
    Temperture_Align_R.setup(robot_config.Temp_Align_R_Close);
    Temperture_Align_L.setup(robot_config.Temp_Align_L_Close);
    sub_mission_sleeping = false;
}

void sub_servo_update(int ms){
    Block_Align_R.Update(ms);
    Block_Align_L.Update(ms);
    Temperture_Align_R.Update(ms);
    Temperture_Align_L.Update(ms);
}

void sub_servo_sleep(){
    // Block_Align_R.detach();
    // Block_Align_L.detach();
    Temperture_Align_R.detach();
    Temperture_Align_L.detach();
    sub_mission_sleeping = true;
}


void submission_executor(void* pvParameters){
    MissionCaller mission_caller;
    const TickType_t xMaxWaitTime = pdMS_TO_TICKS(20000);
    for (;;) {
        if (xQueueReceive(sideQueues[SUBMISSION], &mission_caller, xMaxWaitTime) == pdPASS) {
            sub_servo_init();
            switch (mission_caller.mission_type) {
                case AlignBlock: align_blocks(mission_caller.align_block);  break;
                case PushTemperture: push_temp(mission_caller.push_temp_side,mission_caller.push_temp);  break;
                default: break;
            }
        }
        else{
            sub_servo_sleep();
        }
    }
}

void align_blocks(bool do_align){//1:do,0:exit
    if(do_align){
        Block_Align_R.turnTo(robot_config.r_align_angle_do,0);
        Block_Align_L.turnTo(robot_config.l_align_angle_do,0);
    }else{
        Block_Align_R.turnTo(robot_config.r_align_angle_exit,0);
        Block_Align_L.turnTo(robot_config.l_align_angle_exit,0);
    }
}


void push_temp(SideIndex side,bool to_do){
    if(side == SIDE_R){
        if(to_do){
            Temperture_Align_R.turnTo(robot_config.Temp_Align_R_Open,0);
        }else{
            Temperture_Align_R.turnTo(robot_config.Temp_Align_R_Close,750);
        }
    }else if(side == SIDE_L){
        if(to_do){
            Temperture_Align_L.turnTo(robot_config.Temp_Align_L_Open,0);
        }else{
            Temperture_Align_L.turnTo(robot_config.Temp_Align_L_Close,750);
        }
    }
}
