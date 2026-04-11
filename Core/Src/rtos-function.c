/*
 * rtos-function.cpp
 *
 *  Created on: Mar 1, 2025
 *      Author: stanly
 */

/*stm32 include*/
#include "cmsis_os.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"

/*microROS include*/
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <uxr/client/transport.h>
#include "event_groups.h"


/*user include*/
#include "Debug_Mode.h"
#include "mission.h"
#include "timers.h"
#include "uros_init.h"

/**************** stm32 variable ****************/
extern UART_HandleTypeDef huart1;
/**************** stm32 variable ****************/

extern EventGroupHandle_t MissionUpdateEventGroup;
TimerHandle_t xTimer;

void vTimerCallback(TimerHandle_t xTimer) {
    xEventGroupSetBits(MissionUpdateEventGroup, MISSION_TICK_BIT);
}

/**************** freertos callback ****************/
void StartDefaultTask(void *argument) {
    /*init*/
    mission_init();
    uros_init();
    // rtos timer for refreshing servo angle
    xTimer = xTimerCreate("Servo_Timer", pdMS_TO_TICKS(SERVO_RREFRESH_INTERVAL),pdTRUE, (void *)0, vTimerCallback);
    xTimerStart(xTimer, 0);

    for (;;) {
        if (*is_in_debugger_mode) {
            mission_debugger();
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            uros_agent_status_check();
        }
    }
}
/**************** freertos callback ****************/
