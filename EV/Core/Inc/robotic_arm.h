#ifndef __ROBOTIC_ARM_H__
#define __ROBOTIC_ARM_H__

#include "main.h"
#include "tim.h"
#include "Motor.h"
#include "PS2.h"
#include "math.h"

void Robotic_Arm_Init(void);
void claw_open(void);
void claw_close(void);
void arm_controller(void);

extern float arm_cnt;
extern uint32_t arm_last_time;
extern uint8_t arm_flag;    

#endif 

