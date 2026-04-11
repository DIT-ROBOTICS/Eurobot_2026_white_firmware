/*
 * type_define.h
 *
 *  Created on: Mar 17, 2026
 *      Author: jason
 */

#ifndef INC_TYPE_DEFINE_H_
#define INC_TYPE_DEFINE_H_
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h723xx.h"

#ifdef __cplusplus
}

typedef enum {
    takeup_up,takeup_pull,takeup_check,takeup_down,

    raise_up,raise_check,raise_down,raise_put,raise_init,

    cup_up,cup_down,cup_put,cup_init,

    cascade_up, cascade_down, cascade_init, cascade_ninja,


    SERVO_CONFIG_COUNT
}ServoConfig_Name;

typedef enum  {
    SUBMISSION = 0,
    SIDE_R = 1,
    SIDE_B = 2,
    SIDE_L = 3,
    SIDE_COUNT = 4
}SideIndex;


typedef struct  {
    // 側邊馬達角度組
    int side[SIDE_COUNT][SERVO_CONFIG_COUNT]; // side[1] 為 R, side[2] 為 B, side[3] 為 L

    // 其他全域參數
    int r_align_angle_do, r_align_angle_exit;
    int l_align_angle_do, l_align_angle_exit;
    int Temp_Align_L_Open, Temp_Align_L_Close;
    int Temp_Align_R_Open, Temp_Align_R_Close;
}RobotConfig;



typedef enum {
    TakeUpBlock,
    FlipBlock,
    PutDownBlock,
    CloseMission,
    AlignBlock,
    PushTemperture
} MissionType;


typedef struct {
    MissionType mission_type;
    int pinecone[4];
    bool align_block;
    bool push_temp;
    SideIndex push_temp_side;
}MissionCaller;


typedef struct {
    GPIO_TypeDef * GPIOx;
    uint16_t GPIO_Pin;
}Airpump;

typedef struct {
    GPIO_TypeDef* gpioPort[4];
    uint16_t gpioPin[4];
}MagGroup;


#endif
#endif /* INC_TYPE_DEFINE_H_ */
