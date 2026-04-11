/*
 * ServoArduinoType.cpp
 *
 *  Created on: Aug 12, 2025
 *      Author: jason
 */

#include "ServoArduinoType.h"
#include "config.h"
#include <cmath>



void Servo::setMaxServoSpeed(float rpm){
	servoInfo.maxServoSpeed = rpm;
}

void Servo::setup(int startAngle){
	HAL_TIM_PWM_Start(htim, channel);
	write(startAngle);
	// bbb = 10;
}

void Servo::attach(uint32_t ch) {
	channel = ch;
}

void Servo::setTimer(TIM_HandleTypeDef* h) {
	htim = h;
}

void Servo::setMaxAngle(int ang){
	servoInfo.maxAng = ang;
}

void Servo::setPulseRange(int min,int max){
	servoInfo.maxPulse = max;
	servoInfo.minPulse = min;
}

void Servo::write(int ang) {
	currentAngle = ang;
	int pulseRange = servoInfo.maxPulse - servoInfo.minPulse;
	float pulse = servoInfo.minPulse + (float)pulseRange * (float)ang / (float)servoInfo.maxAng;
	if(pulse > servoInfo.maxPulse)pulse = servoInfo.maxPulse;
	else if(pulse < servoInfo.minPulse)pulse = servoInfo.minPulse;
//	p = pulse;
	__HAL_TIM_SET_COMPARE(htim, channel, int(pulse));
}


void Servo::detach() {
	HAL_TIM_PWM_Stop(htim, channel);
}


int Servo::turnTo(int ang,int interval){
	if (isMoving){ return 0; };
	if(ang > servoInfo.maxAng || ang < 0){ return 0; };
	if (targetAngle == ang){return 0;};

	startAngle  = currentAngle;
	targetAngle  = ang;
	int deltaDeg = ang - currentAngle;
	float minInterval = (float)abs(deltaDeg)*servoInfo.maxServoSpeed/60.0*1000.0*1.1;
	this->durationMs = interval > minInterval ? interval : minInterval;
	elapsedMs = 0;
	isMoving = (deltaDeg != 0);
	return this->durationMs+SERVO_RREFRESH_INTERVAL+1;
}

void Servo::Update(int ms)
{
    if (!isMoving) return;

    elapsedMs += ms;// + 10 - bbb;

    float u = (float)elapsedMs / (float)durationMs;
    if (u >= 1.0f) {
        currentAngle = targetAngle;
        write(currentAngle);
        isMoving = false;
        return;
    }
	float u_smoothed = (1.0f - cosf(u * 3.14159f)) * 0.5f;
    currentAngle = startAngle + (targetAngle - startAngle) * u_smoothed;
    write(currentAngle);
}











