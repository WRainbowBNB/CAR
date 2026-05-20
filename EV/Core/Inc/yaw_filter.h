#ifndef __YAW_FILTER_H__
#define __YAW_FILTER_H__

#include "main.h"
#include "encoder.h"
#include "Motor.h"
#include "imu.h"
#include "Controller.h"

#define PI 3.14159265358979323846f
#define R_ENCODER 0.01 //编码器噪声 待测
#define R_IMU 0.01 //imu噪声 待测
#define PID_T 0.01 //PID周期
#define ALPHA 0.1f //平滑系数，根据实际情况调整
#define RESPONSE_TIME 0.1f //响应时间，根据实际情况调整
#define K_ENCODER1 0.1 //编码器权重 后期可能去掉直接换成卡尔曼
#define K_IMU1 0.9 //imu权重 后期可能直接去掉换成卡尔曼

#define X_R 0.0035f
#define X_ALPHA 0.1f
#define Y_R 0.0035f
#define Y_ALPHA 0.1f
#define Z_R 0.0035f
#define Z_ALPHA 0.1f

//Q小波形平滑，但会有滞后，Q大响应快，但会有毛刺锯齿
//P是预测值误差协方差（滤波器对自己的信任度，越大越不自信），Q是过程噪声，R是测量噪声
//K是权重，根据实际情况调整
typedef struct{
    float P;         //预测值误差协方差
    float P_pre;     //上次预测值误差协方差
    float Q_last;    //上次过程噪声
    float Q_new;     //当前过程噪声
    float w_k;       //最优估计值
    float w_pre;     //预测值
    float error1;    //编码器测量值与预测值的误差
    float error2;    //imu测量值与预测值的误差
    float residual;  //残差
    float M;         //手算完的大分母
    float K_ENCODER; //编码器权重
    float K_IMU;     //imu权重
}Kalman;

typedef struct{
    float target_v;
    float P_pre;
    float P_last;
    float Q_last;
    float Q_new;
    float v_pre;
    float v_k;
}Axis_Kalman;

float encoder_yaw_speed(void);
float yaw_speed_filter(float en_yaw_speed);
float Kalman_Filter(float en_yaw_speed);
float X_Kalman_Filter(float vx);
float Y_Kalman_Filter(float vy);

extern Kalman yaw_filter;
extern Axis_Kalman x_filter;
extern Axis_Kalman y_filter;
extern float Kalman_yaw;

#endif 

