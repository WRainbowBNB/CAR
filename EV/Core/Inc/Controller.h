#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "main.h"
#include "Motor.h"
#include "PS2.h"
#include "stdlib.h"
#include "math.h"
#include "encoder.h"

void Controller(uint8_t *Data);
void Controller2(uint8_t *Data);
float PS2_Kalman_Filter(float vz);

extern float target_vx;
extern float target_vy;
extern float target_vz;

#endif 

