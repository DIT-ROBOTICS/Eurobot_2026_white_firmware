/*
 * subMission_Motion.h
 *
 *  Created on: Apr 8, 2026
 *      Author: jason
 */

#ifndef INC_SUBMISSION_MOTION_H_
#define INC_SUBMISSION_MOTION_H_

#ifdef __cplusplus

#include "type_define.h"

void sub_servo_init();
void sub_servo_update(int ms);
void submission_executor(void* pvParameters);

#endif
#endif /* INC_SUBMISSION_MOTION_H_ */
