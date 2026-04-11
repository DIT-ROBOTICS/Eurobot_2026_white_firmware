/*
 * mission.h
 *
 *  Created on: Mar 2, 2025
 *      Author: stanly
 */

#ifndef INC_MISSION_H_
#define INC_MISSION_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "type_define.h"
#include "Arm_Motion.h"
#define MISSION_TICK_BIT (1 << 0)


void mission_init(void);


extern int process;
extern int is_for_ninja;

#ifdef __cplusplus
}

void mission_deliveryman(int side,MissionCaller *mission_caller);



#endif


#endif /* INC_MISSION_H_ */
