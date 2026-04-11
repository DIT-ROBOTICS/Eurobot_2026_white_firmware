/*
 * Arm_Motion.h
 *
 *  Created on: Apr 7, 2026
 *      Author: jason
 */

#ifndef INC_ARM_MOTION_H_
#define INC_ARM_MOTION_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "type_define.h"

#ifdef __cplusplus
}

#include "ServoArduinoType.h"
#include "config.h"

class Arm{ 
public:

    typedef enum{
        Top,
        Bottom
    }MagPos;

    typedef enum{
        CatchPine,
        ThrowPine
    }FlipAction;

    
    SideIndex side;

private:
    Servo * Arm_TakeUp;
    Servo * Arm_RaiseUp;
    Servo * Arm_SuctionCup;
    Servo * Arm_Cascade;
    MagGroup * MagGroup_Top;
    MagGroup * MagGroup_Bottom;
    Airpump * airpump;
    TickType_t next_available_tick;
    bool is_sleeping;
    int pinecone_stats[4];

    int servo_do(ServoConfig_Name mission,int executeTime);
    void set_MagGroup_stats(MagPos g,int a, int b, int c, int d);
    void set_AirPump_stats(GPIO_PinState stats);

public:
    Arm(SideIndex s, Servo* tu, Servo* ru, Servo* c, Servo* cas, MagGroup* top, MagGroup* bot,Airpump * ap) 
    {
        side = s;
        Arm_TakeUp = tu;
        Arm_RaiseUp = ru;
        Arm_SuctionCup = c;
        Arm_Cascade = cas;
        MagGroup_Top = top;
        MagGroup_Bottom = bot;
        airpump = ap;
        next_available_tick = xTaskGetTickCount();//xTaskGetTickCount' was not declared in this scope
        is_sleeping = false;
    }

    void init();
    void update(int ms = SERVO_RREFRESH_INTERVAL);
    void sleep();
    void wake_up();
    void set_pinecone_stats(int *p);
    int* get_pinecone_stats();
    void do_and_wait(ServoConfig_Name name,int time = 0, int extra_ms = SERVO_RREFRESH_INTERVAL);
    void do_async(ServoConfig_Name name,int time = 0, int extra_ms = SERVO_RREFRESH_INTERVAL);
    void sync(int extra_ms = 0);
    void arm_default_pose();
    void servo_init_pose(int wait = 0);
    void MagGroup_init_pose();
    void mag_take_pinecone(MagPos g);
    void mag_put_pinecone(MagPos g,int wait = 0);
    void mag_flip_pinecone(MagPos g,FlipAction a,int wait = 0);
    bool is_all_right();
    bool is_all_wrong();
};


extern Arm * arms[];

#endif

#endif /* INC_ARM_MOTION_H_ */
