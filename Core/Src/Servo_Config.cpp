/*
 * Servo_Config.cpp
 *
 *  Created on: Feb 11, 2026
 *      Author: jason
 */




#include "Servo_Config.h"
#include "stm32h7xx_hal.h"
#include "ServoArduinoType.h"


extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim23;

// 第一面（面向電腦的左邊）：
Servo R_TakeUp_Block            = {&htim8,TIM_CHANNEL_4,Servo::GBD300T};
Servo R_RaiseUp_Block           = {&htim8,TIM_CHANNEL_3,Servo::GBD300T};
Servo R_SuctionCup_Ang_Ctrl     = {&htim15,TIM_CHANNEL_1,Servo::S9IMOD11KG};
Servo R_Cascade                 = {&htim23,TIM_CHANNEL_3,Servo::GBD1800SS};
MagGroup R_Top_Group    = {{GPIOG,GPIOG,GPIOG,GPIOD},{GPIO_PIN_14,GPIO_PIN_13,GPIO_PIN_12,GPIO_PIN_2}};
MagGroup R_Bottom_Group = {{GPIOG,GPIOG,GPIOD,GPIOD},{GPIO_PIN_10,GPIO_PIN_9,GPIO_PIN_4,GPIO_PIN_3}};
Airpump R_Airpump               = {GPIOG, GPIO_PIN_11};

Arm RightSideArm = Arm(SIDE_R,&R_TakeUp_Block,&R_RaiseUp_Block,&R_SuctionCup_Ang_Ctrl,&R_Cascade,&R_Top_Group,&R_Bottom_Group,&R_Airpump);


// 第二面（面向電腦的後面）：
Servo B_TakeUp_Block            = {&htim4,TIM_CHANNEL_4,Servo::GBD300T};
Servo B_RaiseUp_Block           = {&htim4,TIM_CHANNEL_3,Servo::GBD300T};
Servo B_SuctionCup_Ang_Ctrl     = {&htim8,TIM_CHANNEL_1,Servo::S9IMOD11KG};
Servo B_Cascade                 = {&htim23,TIM_CHANNEL_4,Servo::GBD1800SS};
MagGroup B_Top_Group    = {{GPIOD,GPIOD,GPIOG,GPIOG},{GPIO_PIN_1,GPIO_PIN_0,GPIO_PIN_8,GPIO_PIN_2}};
MagGroup B_Bottom_Group = {{GPIOG,GPIOG,GPIOG,GPIOG},{GPIO_PIN_6,GPIO_PIN_5,GPIO_PIN_4,GPIO_PIN_3}};
Airpump B_Airpump               = {GPIOG, GPIO_PIN_7};

Arm BackSideArm = Arm(SIDE_B,&B_TakeUp_Block,&B_RaiseUp_Block,&B_SuctionCup_Ang_Ctrl,&B_Cascade,&B_Top_Group,&B_Bottom_Group,&B_Airpump);


// 第三面（面向電腦的右邊）：
Servo L_TakeUp_Block            = {&htim5,TIM_CHANNEL_4,Servo::GBD300T};
Servo L_RaiseUp_Block           = {&htim5,TIM_CHANNEL_3,Servo::GBD300T};
Servo L_SuctionCup_Ang_Ctrl     = {&htim4,TIM_CHANNEL_1,Servo::S9IMOD11KG};
Servo L_Cascade                 = {&htim5,TIM_CHANNEL_1,Servo::GBD1800SS};
MagGroup L_Top_Group    = {{GPIOE,GPIOE,GPIOE,GPIOE},{GPIO_PIN_15,GPIO_PIN_14,GPIO_PIN_13,GPIO_PIN_7}};
MagGroup L_Bottom_Group = {{GPIOE,GPIOE,GPIOE,GPIOE},{GPIO_PIN_11,GPIO_PIN_10,GPIO_PIN_9,GPIO_PIN_8}};
Airpump L_Airpump               = {GPIOE, GPIO_PIN_12};

Arm LeftSideArm = Arm(SIDE_L,&L_TakeUp_Block,&L_RaiseUp_Block,&L_SuctionCup_Ang_Ctrl,&L_Cascade,&L_Top_Group,&L_Bottom_Group,&L_Airpump);



Arm * arms[] = {nullptr,&RightSideArm,&BackSideArm,&LeftSideArm};

// other:
Servo Block_Align_R             = {&htim8,TIM_CHANNEL_2,Servo::S9IMOD11KG};
Servo Block_Align_L             = {&htim4,TIM_CHANNEL_2,Servo::S9IMOD11KG};
Servo Temperture_Align_R        = {&htim5,TIM_CHANNEL_2,Servo::S9IMOD11KG};
Servo Temperture_Align_L        = {&htim23,TIM_CHANNEL_1,Servo::S9IMOD11KG};

RobotConfig robot_config;

void write_in_robot_config(void){

    robot_config.side[SIDE_R][takeup_up]      = 194;
    robot_config.side[SIDE_R][takeup_pull]    = 190;
    robot_config.side[SIDE_R][takeup_check]   = 145;
    robot_config.side[SIDE_R][takeup_down]    = 100;
    robot_config.side[SIDE_R][raise_up]       = 205;
    robot_config.side[SIDE_R][raise_check]    = 135;
    robot_config.side[SIDE_R][raise_down]     = 155;
    robot_config.side[SIDE_R][raise_put]      = 180;
    robot_config.side[SIDE_R][raise_init]     = 120;
    robot_config.side[SIDE_R][cup_up]         = 112;
    robot_config.side[SIDE_R][cup_down]       = 63;
    robot_config.side[SIDE_R][cup_put]        = 92;
    robot_config.side[SIDE_R][cup_init]       = 150;
    robot_config.side[SIDE_R][cascade_init]   = 660;
    robot_config.side[SIDE_R][cascade_up]     = 680;
    robot_config.side[SIDE_R][cascade_down]   = 1350;
    robot_config.side[SIDE_R][cascade_ninja]  = 800;


    robot_config.side[SIDE_B][takeup_up]      = 193;
    robot_config.side[SIDE_B][takeup_pull]    = 190;
    robot_config.side[SIDE_B][takeup_check]   = 145;
    robot_config.side[SIDE_B][takeup_down]    = 100;
    robot_config.side[SIDE_B][raise_up]       = 198;
    robot_config.side[SIDE_B][raise_check]    = 135;
    robot_config.side[SIDE_B][raise_down]     = 155;
    robot_config.side[SIDE_B][raise_put]      = 180;
    robot_config.side[SIDE_B][raise_init]     = 120;
    robot_config.side[SIDE_B][cup_up]         = 103;
    robot_config.side[SIDE_B][cup_down]       = 53;
    robot_config.side[SIDE_B][cup_put]        = 80;
    robot_config.side[SIDE_B][cup_init]       = 138;
    robot_config.side[SIDE_B][cascade_init]   = 666;
    robot_config.side[SIDE_B][cascade_up]     = 750;
    robot_config.side[SIDE_B][cascade_down]   = 1190;
    robot_config.side[SIDE_B][cascade_ninja]  = 1110;



    robot_config.side[SIDE_L][takeup_up]      = 195;
    robot_config.side[SIDE_L][takeup_pull]    = 191;
    robot_config.side[SIDE_L][takeup_check]   = 145;
    robot_config.side[SIDE_L][takeup_down]    = 95;
    robot_config.side[SIDE_L][raise_up]       = 203;
    robot_config.side[SIDE_L][raise_check]    = 143;
    robot_config.side[SIDE_L][raise_down]     = 150;
    robot_config.side[SIDE_L][raise_put]      = 185;
    robot_config.side[SIDE_L][raise_init]     = 120;
    robot_config.side[SIDE_L][cup_up]         = 95;
    robot_config.side[SIDE_L][cup_down]       = 50;
    robot_config.side[SIDE_L][cup_put]        = 65;
    robot_config.side[SIDE_L][cup_init]       = 135;
    robot_config.side[SIDE_L][cascade_init]   = 700;
    robot_config.side[SIDE_L][cascade_up]     = 740;
    robot_config.side[SIDE_L][cascade_down]   = 1240;
    robot_config.side[SIDE_L][cascade_ninja]  = 1190;





    robot_config.r_align_angle_do   = 133;
    robot_config.r_align_angle_exit = 20;
    robot_config.l_align_angle_do   = 47;
    robot_config.l_align_angle_exit = 160;
    robot_config.Temp_Align_L_Open  = 95;
    robot_config.Temp_Align_L_Close = 136;
    robot_config.Temp_Align_R_Open  = 95;
    robot_config.Temp_Align_R_Close = 50;
}

