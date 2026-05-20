#ifndef __MOTOR_H__
#define __MOTOR_H__


#include "main.h"
#include "tim.h"
#include "math.h"
#include "encoder.h"
#include "Controller.h"
#include "yaw_filter.h"


#define MAX_PWM 90
#define MAX_ANGULAR 3.14f //单位：rad/s
#define MAX_SPEED 110.0f //单位：cm/s
#define PWM_SCALE ((float)MAX_PWM / (float)MAX_SPEED) //速度转PWM比例
#define DX 0.0  //重心X偏移量
#define DY 0.0  //重心Y偏移量
#define A 10.0f //单位：cm 车轴到几何中心X轴距离
#define B 7.5f  //单位：cm 车轴到几何中心Y轴距离
#define k_fl ((A + DX) + (B - DY)) //左前
#define k_fr ((A - DX) + (B - DY)) //右前
#define k_rl ((A + DX) + (B + DY)) //左后
#define k_rr ((A - DX) + (B + DY)) //右后
#define LIMIT_PWM(x) ((double)(x) > MAX_PWM ? MAX_PWM : (double)(x) < -MAX_PWM ? -MAX_PWM : (double)(x))


void MOTOR_Init(void);
void Motor1_SetSpeed(int16_t Speed);
void Motor2_SetSpeed(int16_t Speed);
void Motor3_SetSpeed(int16_t Speed);
void Motor4_SetSpeed(int16_t Speed);
void Motor_Stop(void);
void Mecanum_SetSpeed(float vx, float vy, float vz);
void Mecanum_Solver(void); //麦轮逆运动学解算，计算各轮目标速度并输出PWM
void Mecanum_ReadSpeed(void);

extern float en_fl_speed;
extern float en_fr_speed;
extern float en_rl_speed;
extern float en_rr_speed;

#endif 

