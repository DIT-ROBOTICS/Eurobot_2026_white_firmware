/*
 * Servo.h
 *
 *  Created on: Mar 7, 2025
 *      Author: jason
 */

/*timer 設定：
  Prescaler = 16-1;
  Period = 19999;
*/
#ifndef INC_SERVOARDUINOTYPE_H_
#define INC_SERVOARDUINOTYPE_H_
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
}
struct Info{
	int maxAng;
	int minPulse;
	int maxPulse;
	float maxServoSpeed;//rpm sec/60˚
};

class Servo {
	public:
		enum type{
			GBD300T,
			GBD300S,
			GBD1800T,
			GBD1800S,
			GBD1800SS,
			sv1232MG,
			S9IMOD11KG,
			GBDHL30mm,
		};


	private:
		uint32_t channel;
		TIM_HandleTypeDef* htim;

		Info servoInfo = {300,500,2500,0.25};

		enum type ServoType = GBD300T;

		int startAngle = 0;
		int targetAngle = 0;
		int currentAngle = 0;
		int durationMs = 0;
		int elapsedMs = 0;
		bool isMoving = false;

		static Info getServoInfo(type servoType){
			switch(servoType){
			case GBD300T:
				return {300,500,2500,0.25};
			case GBD300S:
				return {300,500,2500,0.11};
			case GBD1800T:
				return {1800,500,2500,0.25};
			case GBD1800S:
				return {1800,500,2500,0.07};
			case GBD1800SS:
				return {1800,500,2500,0.035};
			case sv1232MG:
				return {165,800,2200,0.05};
			case S9IMOD11KG:
				return {180,500,2500,0.08};
			case GBDHL30mm:
				return {30,1050,1950,2.0};
			default:
				return getServoInfo(GBD300T);
			}
		}


	public:

		Servo(TIM_HandleTypeDef* h, uint32_t ch,int MaxAngle = 300,int MinPulse = 500,int MaxPulse = 2500,int maxServoSpeed = 0.25) {
			htim = h;
			channel = ch;
			servoInfo.maxAng = MaxAngle;
			servoInfo.minPulse = MinPulse;
			servoInfo.maxPulse = MaxPulse;
			servoInfo.maxServoSpeed = maxServoSpeed;
		}

		Servo(TIM_HandleTypeDef* h, uint32_t ch,type servoType) {
			htim = h;
			channel = ch;
			ServoType = servoType;
			servoInfo = getServoInfo(servoType);
		}
		void setMaxServoSpeed(float rpm);
		void setup(int startAngle = 0);
		void attach(uint32_t ch);
		void setTimer(TIM_HandleTypeDef* h);
		void setMaxAngle(int ang);
		void setPulseRange(int min,int max);
		void write(int ang);//直接轉到對的位置
		int  turnTo(int ang,int interval);//會檢查有沒有轉到 interval單位是ms
		void detach();
		void Update(int ms);
};

#endif
#endif /* INC_SERVO_H_ */
