/*
 * mission.cpp
 *
 *  Created on: Mar 2, 2026
 *      Author: Jason
 */

/* 1. 核心邏輯頭文件 */
#include "mission.h"

/* 2. C 標準庫 (用於字串處理、隨機數) */
#include <string.h>
#include <cstdio>
#include <stdlib.h>

/* 3. STM32 與 RTOS 核心 */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "config.h"

/* 4. 項目特定模組 (由底層到高層) */
#include "type_define.h"
#include "subMission_Motion.h"
#include "Servo_Config.h"
#include "uros_config.h"
#include "Debug_Mode.h"


/*for debugger mode*/
int process = 0;
int is_for_ninja = 0;
/*for debugger mode*/


/* for mission call controllor */

QueueHandle_t sideQueues[SIDE_COUNT];

EventGroupHandle_t MissionUpdateEventGroup;
//
TaskHandle_t All_xTask[5];
//UBaseType_t remainingStack[5];

/* for mission call controllor */

void mission_updater(void *argument);
void mission_executor(void* pvParameters);
void submission_executor(void* pvParameters);

void take_up_block(Arm * arm);
void flip_block(Arm * arm);
void put_down_block(Arm * arm);
void take_up_block_ninja(Arm * arm);
void flip_block_ninja(Arm * arm);




void mission_init(void){
    write_in_robot_config();
    MissionUpdateEventGroup = xEventGroupCreate();
    xTaskCreate(mission_updater, "MissionUpdate", 512, NULL, osPriorityHigh, &All_xTask[4]);
    for(int i = SUBMISSION; i < SIDE_COUNT; i++) {
        sideQueues[i] = xQueueCreate(5, sizeof(MissionCaller));
        // 為每一邊啟動一個獨立的 Task
        // 使用不同的名稱與參數，讓 Task 知道自己負責哪一邊
        char taskName[32] = "mission_status_checker";
        sprintf(taskName, "mission_status_checker_%d", i);
        // 傳入 i 作為參數 (pvParameters)
        if(i == SUBMISSION){
            xTaskCreate(submission_executor, taskName, 512, (void*)(uintptr_t)i, osPriorityBelowNormal, &All_xTask[i]);
            continue;
        }
        arms[i]->init();
        xTaskCreate(mission_executor, taskName, 512, (void*)(uintptr_t)i, osPriorityNormal, &All_xTask[i]);
        arms[i]->arm_default_pose();
    }
    sub_servo_init();
}

void mission_updater(void *argument){
    for (;;)
    {
        xEventGroupWaitBits(
        	MissionUpdateEventGroup,
            MISSION_TICK_BIT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );
        for(int i = SIDE_R; i < SIDE_COUNT; i++){
            arms[i]->update(SERVO_RREFRESH_INTERVAL);
        }
        sub_servo_update(SERVO_RREFRESH_INTERVAL);
    }
}

void mission_deliveryman(int side,MissionCaller *mission_caller){
    if(side < SUBMISSION || side > SIDE_L)return;
    SideIndex s = static_cast<SideIndex>(side);
    if (xQueueSend(sideQueues[s], mission_caller, 0) != pdPASS) {
        // 佇列滿了的處理邏輯
    }
    
//    for(int i = 0; i < 5; i++){
//        remainingStack[i] = uxTaskGetStackHighWaterMark(All_xTask[i]);
//    }
}

void mission_executor(void* pvParameters) {
    SideIndex mySide = (SideIndex)(uintptr_t)pvParameters;
    Arm * arm = arms[mySide];
    MissionCaller mission_caller;
    const TickType_t xMaxWaitTime = pdMS_TO_TICKS(AutoSleepThreshold);

    for (;;) {
        if (xQueueReceive(sideQueues[mySide], &mission_caller, xMaxWaitTime) == pdPASS) {
            arm -> wake_up();
            switch (mission_caller.mission_type) {
                case TakeUpBlock: 
                    arm -> set_pinecone_stats(mission_caller.pinecone);
                    if(is_for_ninja == mySide) take_up_block_ninja(arm);
                    else take_up_block(arm); 
                    break;
                case FlipBlock: 
                    if(is_for_ninja == mySide)  flip_block_ninja(arm) ;
                    else flip_block(arm); 
                    break;
                case PutDownBlock:  put_down_block(arm);       break;
                case CloseMission:   arm->arm_default_pose(); break;
                default: break;
            }
        } else  {
            arm -> arm_default_pose();
            arm -> sleep();
        }
    }
}



void take_up_block(Arm * arm){
    process = 1000;
    arm -> servo_init_pose(1);
    arm -> MagGroup_init_pose();
    arm -> mag_take_pinecone(Arm::Bottom);
    arm -> do_and_wait(cup_init, 0);
    arm -> do_async(takeup_down, 0);
    arm -> do_async(raise_check, 250);
    arm -> sync(100);
    arm -> do_async(takeup_check,0);
    process = process/10*10+1;
    process = finish_take(arm -> side);
    arm -> sync(1000);
    arm -> do_and_wait(raise_up,0);
    arm -> servo_init_pose(1);
    process = process/10*10+2;
}

void flip_block(Arm * arm){
    process = 2000;
    if(arm -> is_all_right()){
        process = finish_flip(arm -> side);
        return;
    }
    arm -> servo_init_pose(1);
    arm -> do_async(cup_down,300);
    arm -> mag_flip_pinecone(Arm::Top,Arm::CatchPine,200);
    arm -> do_and_wait(cascade_up,0,100);
    arm -> sync(0);
    arm -> do_and_wait(raise_down,400);

    arm -> mag_flip_pinecone(Arm::Bottom,Arm::ThrowPine,0);
    arm -> do_and_wait(takeup_pull,0,400);

    arm -> do_async(takeup_up,0);
    arm -> do_async(cascade_init,0);
    arm -> do_async(cup_up,500);
    arm -> do_async(raise_up,0);

    process = process/10*10+1;
    process = finish_flip(arm -> side);
    arm -> sync(10);
    process = process/10*10+2;
}

void put_down_block(Arm * arm){
    process = 3000;
    arm -> servo_init_pose(0);

    if(!arm -> is_all_wrong()) arm -> do_and_wait(takeup_down,0,100);

    if(!arm -> is_all_right()) arm -> do_async(raise_put,500);

    if(!arm -> is_all_wrong()){
        arm -> mag_put_pinecone(Arm::Bottom,50);
        arm -> do_and_wait(takeup_up,0,50);
    }

    if(arm -> is_all_right()){
        arm -> arm_default_pose();
        process = finish_put(arm -> side);
        return;
    }

    arm -> sync(0);
    arm -> do_and_wait(cascade_down,0);
    arm -> do_and_wait(cup_put,200,200);
    arm -> mag_put_pinecone(Arm::Top,100);

    process = process/10*10+1;
    process = finish_put(arm -> side);

    arm -> servo_init_pose(1);
    arm -> arm_default_pose();

    process = process/10*10+2;
}



void take_up_block_ninja(Arm * arm){
    // process = 0;
    // mission_initial_pose(side,0);
    // mag_val_initial_pose(side);
    // int servo_RaiseUp_sem = servo_do(raise_put,side,0);
    // int servo_CupAngle_sem = servo_do(cup_up,side,0);
    // osDelay(std::max({servo_RaiseUp_sem,servo_CupAngle_sem})+ServoRefreshInterval);
    // int servo_Cascade_sem = servo_do(cascade_ninja,side,0);
    // osDelay(servo_Cascade_sem+300);
    // servo_Cascade_sem = servo_do(cascade_init,side,500);
    // servo_RaiseUp_sem = servo_do(raise_up,side,servo_Cascade_sem);
    // osDelay(std::max({servo_Cascade_sem,servo_RaiseUp_sem})+ServoRefreshInterval);
    // process = 1;
}

void flip_block_ninja(Arm * arm){
    // process = 2;
    // int p0 = pinecone[0];
    // int p1 = pinecone[1];
    // int p2 = pinecone[2];
    // int p3 = pinecone[3];
    // mission_initial_pose(side,1);
    // mag_val_initial_pose(side);
    // set_mag_BottomGroup_stats(side,!p0,!p1,!p2,!p3);
    // int servo_Cascade_sem = servo_do(cascade_init,side,0);
    // int servo_CupAngle_sem = servo_do(cup_down,side,500);
    // osDelay(servo_CupAngle_sem+10);
    // int servo_RaiseUp_sem = servo_do(raise_down,side,500);
    // osDelay(std::max({servo_Cascade_sem,servo_CupAngle_sem,servo_RaiseUp_sem})+100);
    // // int servo_TakeUp_sem = servo_TakeUp_suck(side,0);
    // set_mag_TopGroup_stats(side,p0,p1,p2,p3);
    // osDelay(300);
    // // servo_TakeUp_sem = servo_TakeUp_up(side,0);
    // servo_Cascade_sem = servo_do(cascade_init,side,0);
    // servo_CupAngle_sem = servo_do(cup_up,side,500);
    // servo_RaiseUp_sem = servo_do(raise_up,side,0);

    // osDelay(std::max({servo_Cascade_sem,servo_CupAngle_sem,servo_RaiseUp_sem/*,servo_TakeUp_sem*/})+ServoRefreshInterval);
    // process = 3;
}
