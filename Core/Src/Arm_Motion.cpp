/*
 * Arm_Motion.cpp
 *
 *  Created on: Apr 7, 2026
 *      Author: jason
 */

#include "Arm_Motion.h"

#include <string.h>
#include "Servo_Config.h"

static int flip(int x);
static int off(int x);


void Arm::init() {
    if (!Arm_TakeUp) return;
    // 使用 robot_config 內的數據初始化角度
    Arm_TakeUp->setup(robot_config.side[side][takeup_up]);
    Arm_RaiseUp->setup(robot_config.side[side][raise_init]);
    Arm_SuctionCup->setup(robot_config.side[side][cup_init]);
    Arm_Cascade->setup(robot_config.side[side][cascade_init]);
    
    // 電磁閥重置
    MagGroup_init_pose();
    set_AirPump_stats(GPIO_PIN_SET);

    is_sleeping = false;
}

void Arm::update(int ms){
    Arm_TakeUp -> Update(ms);
    Arm_RaiseUp -> Update(ms);
    Arm_SuctionCup -> Update(ms);
    Arm_Cascade -> Update(ms);
}

void Arm::sleep(){
    Arm_TakeUp -> detach();
    Arm_RaiseUp -> detach();
    Arm_SuctionCup -> detach();
    Arm_Cascade -> detach();
    MagGroup_init_pose();
    set_AirPump_stats(GPIO_PIN_RESET);
    is_sleeping = true;
}

void Arm::wake_up(){
    if(is_sleeping)init();
}

void Arm::set_pinecone_stats(int *p){
    memcpy(pinecone_stats, p, sizeof(int) * 4);
}

int * Arm::get_pinecone_stats(){
    return pinecone_stats;
}

int Arm::servo_do(ServoConfig_Name mission,int executeTime){
    switch(mission){
        case takeup_up:
        case takeup_pull:
        case takeup_check:
        case takeup_down:
            return Arm_TakeUp->turnTo(robot_config.side[side][mission],executeTime);
        case raise_up:
        case raise_down:
        case raise_check:
        case raise_put:
        case raise_init:
            return Arm_RaiseUp->turnTo(robot_config.side[side][mission],executeTime);

        case cup_up:
        case cup_down:
        case cup_put:
        case cup_init:
            return Arm_SuctionCup->turnTo(robot_config.side[side][mission],executeTime);

        case cascade_up:
        case cascade_down:
        case cascade_init:
        case cascade_ninja:
            return Arm_Cascade->turnTo(robot_config.side[side][mission],executeTime);

        default:
            return 0;
    }
}


void Arm::do_and_wait(ServoConfig_Name name, int time, int extra_ms) {
    // 1. 執行動作並取得該動作需要的時間
    int duration = servo_do(name, time);
    
    // 2. 計算「這個動作」預計完成的時間點
    TickType_t this_action_finish = xTaskGetTickCount() + pdMS_TO_TICKS(duration + extra_ms);

    vTaskDelay(pdMS_TO_TICKS(duration + extra_ms));
    
    // 3. 更新全域時間軸：取「原本最晚」與「現在這個動作」的極大值
    // 這樣才不會刷掉之前 do_async 留下的長延遲
    if (this_action_finish > next_available_tick) {
        next_available_tick = this_action_finish;
    }
}

void Arm::do_async(ServoConfig_Name name,int time, int extra_ms) {
    int duration = servo_do(name, time);
    TickType_t finish_at = xTaskGetTickCount() + pdMS_TO_TICKS(duration + extra_ms);
    if (finish_at > next_available_tick) {
        next_available_tick = finish_at;
    }
    vTaskDelay(pdMS_TO_TICKS(extra_ms));
}


void Arm::sync(int extra_ms) {
    TickType_t now = xTaskGetTickCount();
    if (next_available_tick > now) vTaskDelay(next_available_tick - now);
    vTaskDelay(pdMS_TO_TICKS(extra_ms));
    next_available_tick = xTaskGetTickCount();
}

void Arm::set_MagGroup_stats(MagPos g,int a, int b, int c, int d){
    GPIO_PinState mag[] = {
        (a ? GPIO_PIN_SET : GPIO_PIN_RESET),
        (b ? GPIO_PIN_SET : GPIO_PIN_RESET),
        (c ? GPIO_PIN_SET : GPIO_PIN_RESET),
        (d ? GPIO_PIN_SET : GPIO_PIN_RESET)
    };
    if(g == Top){
        for(int i = 0; i < 4; i++){
            HAL_GPIO_WritePin(MagGroup_Top -> gpioPort[i], MagGroup_Top -> gpioPin[i], mag[i]);
        }
    }else if(g == Bottom){
        for(int i = 0; i < 4; i++){
            HAL_GPIO_WritePin(MagGroup_Bottom -> gpioPort[i], MagGroup_Bottom -> gpioPin[i], mag[i]);
        }
    }
}
void Arm::set_AirPump_stats(GPIO_PinState stats){
    HAL_GPIO_WritePin(airpump -> GPIOx, airpump -> GPIO_Pin, stats);
}

void Arm::MagGroup_init_pose(){
    set_MagGroup_stats(Top,0,0,0,0);
    set_MagGroup_stats(Bottom,0,0,0,0);
}

void Arm::arm_default_pose(){
    MagGroup_init_pose();
    do_and_wait(cascade_init,0);
    do_async(cup_init,0);
    do_async(raise_init,0);
    do_async(takeup_up,0);
    sync(100);
}


void Arm::servo_init_pose(int wait){
    do_async(takeup_up,side,0);
    do_async(raise_up,side,0);
    do_async(cup_up,side,0);
    do_async(cascade_init,side,0);
    if(wait)sync(100);
}


void Arm::mag_take_pinecone(Arm::MagPos g){
    int p0 = pinecone_stats[0];
    int p1 = pinecone_stats[1];
    int p2 = pinecone_stats[2];
    int p3 = pinecone_stats[3];
    set_MagGroup_stats(g,off(p0),off(p1),off(p2),off(p3));
}


void Arm::mag_put_pinecone(Arm::MagPos g,int wait){
    set_MagGroup_stats(g,1,1,1,1);
    vTaskDelay(pdMS_TO_TICKS(wait));
}
void Arm::mag_flip_pinecone(Arm::MagPos g,Arm::FlipAction a,int wait){
    int p0 = pinecone_stats[0];
    int p1 = pinecone_stats[1];
    int p2 = pinecone_stats[2];
    int p3 = pinecone_stats[3];
    if(a == Arm::CatchPine){
        set_MagGroup_stats(g,flip(p0),flip(p1),flip(p2),flip(p3));
    }
    else if(a == Arm::ThrowPine){
        set_MagGroup_stats(g,p0,p1,p2,p3);
    }
}


bool Arm::is_all_right(){
    for (int i = 0; i < 4; i++) {
        if (pinecone_stats[i] > 0) {
            return false;
        }
    }
    return true;
}

bool Arm::is_all_wrong(){
    for (int i = 0; i < 4; i++) {
        if (pinecone_stats[i] < 1) {
            return false;
        }
    }
    return true;
}


static int flip(int x) {
    return (x == -1) ? x : !x;
}

static int off(int x){
    return (x == -1) ? x : 0;
}
