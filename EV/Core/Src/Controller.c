#include "Controller.h"

#define PS2_R 0.0035f
#define PS2_ALPHA 0.1f

float target_vx = 0.0f;
float target_vy = 0.0f;
float target_vz = 0.0f;

float PS2_P_pre = 0.0f;
float PS2_P_last = 1.0f;
float PS2_Q_last = 0.01f;
float PS2_Q_new = 0.0f;
float PS2_w_pre = 0.0f;
float PS2_w_k = 0.0f;

//tips: Data[7] && [8]
void Controller(uint8_t *Data)
{

    int16_t X = (int16_t)Data[7] - 128;
    int16_t Y = (int16_t)128 - Data[8];
    int16_t Z = (int16_t)Data[5] - 128;

    if(abs(X) < 20) X = 0;
    if(abs(Y) < 20) Y = 0;
    if(abs(Z) < 20) Z = 0;

    if(Y < 0) X = -X;   //避免倒车镜像，笑拉了倒反了

    int16_t left_speed = LIMIT_PWM(Y + X * 0.8 + Z);
    int16_t right_speed = LIMIT_PWM(Y - X * 0.8 - Z);

    Motor1_SetSpeed(left_speed);
    Motor2_SetSpeed(right_speed);
    Motor3_SetSpeed(left_speed);
    Motor4_SetSpeed(right_speed);

}
	
//以下是麦轮控制

void Controller2(uint8_t *Data)
{
    int16_t X = 0;
    int16_t Y = 0;
    int16_t Z = 0;
    if(last_PS2_flag != 0){
        // 没收着，收的不对，单片机似了，停车（主要是为了防失控）
        X = (int16_t)(Data[7] - 128);
        Y = (int16_t)(128 - Data[8]);
        Z = (int16_t)(Data[5] - 128);
    }
    
    if(abs(X) < 20) X = 0;
    if(abs(Y) < 20) Y = 0;
    if(abs(Z) < 20) Z = 0;

    //车体期望速度
    float vx = (float)X / 128.0f * MAX_SPEED;    //左右
    float vy = (float)Y / 128.0f * MAX_SPEED;    //前后
    float vz = -(float)Z / 128.0f * MAX_ANGULAR;  //旋转
    //注意这里加负号是为了符合直觉，因为逆正顺负，即左转是正数，右转是负数
    //使用卡尔曼滤波旋转速度
    vz = PS2_Kalman_Filter(vz);
    
    Mecanum_SetSpeed(vx, vy, vz);
}

float PS2_Kalman_Filter(float vz){
    //预测阶段
    PS2_w_pre = PS2_w_k;
    PS2_P_pre = PS2_P_last + PS2_Q_last;
    float residual = vz - PS2_w_pre;
    PS2_Q_new = (1- PS2_ALPHA) * PS2_Q_last + PS2_ALPHA * residual * residual;
    if(PS2_Q_new < 0.00001f) PS2_Q_new = 0.00001f;
    //修正阶段
    float K = PS2_P_pre / (PS2_P_pre + PS2_R);
    PS2_w_k = PS2_w_pre + K * residual;
    PS2_P_last = (1 - K) * PS2_P_pre;
    PS2_Q_last = PS2_Q_new;
    return PS2_w_k;
}