#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"
#include "Motor.h"
#include "math.h"
#include "string.h"
#include "tim.h"
#include "imu.h"
#include "LineTracking.h"
#include "yaw_filter.h"


#define MEDIAN_WINDOW_SIZE 3     //中值滤波窗口大小
#define MAX_SPEED_DIFF     6.0f  //最大速度差（cm/s）
#define STOP_THRESHOLD     60.0f //停止阈值（cm/s）
#define PID_DT             0.01f //PID计算周期10ms
#define MOTOR_REDUCTION    30.0f //减速比
#define ENCODER_LINES      11.0f //编码器线数
#define MOTOR_DEADZONE     0.0f //死区补偿PWM值
#define WHEEL_CIRCUMFERENCE 25.12f //轮子周长（cm）
#define MAX_RPM 333.0f //最大转速（r/min）
#define dpos 0.07615f //单位：cm/脉冲
//*****PID与滤波结构体（纯算法）******//

typedef struct{
    //参数
    float kp;
    float ki;
    float kd;
    float MAXOutput;    //输出限幅
    float MAXIntegral;  //积分限幅

    //运行时状态
    float target_speed;       //目标值
    float current_speed;       //当前实际值
    float error;        //当前误差
    float lasterror;    //上一次误差
    float integral_sum;  //积分项累加值
    float output;       //PID输出值
    int stop_timeout_cnt; //异常周期数
    int capture_flag;
    float pos;

    //微分滤波相关
    float speed_kd;
    float filtered_speed_kd;   //滤波后的kd

    //限幅滤波相关
    float last_valid_speed; //上一次有效速度值
    int filter_noise_cnt; //噪声计数

    //中值滤波相关
    float speed_buffer[MEDIAN_WINDOW_SIZE]; //中值滤波缓冲区
    uint8_t speed_buffer_index; //当前缓冲区索引
}PIDController;

//********电机硬件句柄结构体（硬件映射）*********//
typedef struct{
    //包含一个PID控制器对象
    PIDController pid;

    //硬件接口
    //电机
    TIM_HandleTypeDef *PWMtim; //PWM定时器句柄
    uint32_t pwm_channel;      //PWM定时器通道
    GPIO_TypeDef *dir_port1;   //方向引脚端口
    uint16_t dir_pin1;         //方向引脚编号1
    GPIO_TypeDef *dir_port2;   //方向引脚端口
    uint16_t dir_pin2;         //方向引脚编号2
    //编码器
    TIM_HandleTypeDef *enc_tim;//编码器定时器句柄
    uint32_t enc_channel;      //编码器定时器通道(单相)
    GPIO_TypeDef *enc_port1;   //编码器引脚端口
    uint16_t enc_pin1;         //编码器引脚编号1
    GPIO_TypeDef *enc_port2;   //编码器引脚端口
    uint16_t enc_pin2;         //编码器引脚编号2

    //编码器数据
    volatile int32_t pulse_cnt; //脉冲数
    volatile float interval;      //时间间隔（单位：ms）
    volatile int overflow_cnt;    //定时器溢出计数（用于计算时间间隔)
    int last_capture_val;    //上一次捕获值
    int cur_capture_val;     //当前捕获值
    int cur_direction; //当前旋转方向,1：正转，-1：反转

}Motor_Handle;

//*********位置环*********//
typedef struct{
    float X_world; //世界坐标系X轴位置
    float Y_world; //世界坐标系Y轴位置
}Position;
typedef struct{
    Position position;
    //参数
    float kp[2];
    float ki[2];
    float kd[2];
    //运行时状态
    //[0]:X轴
    //[1]:Y轴
    float target_pos[2];
    float current_pos[2];
    float error[2];
    float lasterror[2];
    float integral_sum[2];  //积分项累加值
    float output[2];       //PID输出值
    float angle;        //车体坐标系与世界坐标系的夹角
    float dx_body;           //车体坐标系X轴微位移
    float dy_body;           //车体坐标系Y轴微位移
    float dx_world;          //世界坐标系X轴微位移
    float dy_world;          //世界坐标系Y轴微位移
    float MAXIntegral[2];  //积分限幅
    float MAXOutput[2];    //输出限幅
    float speed_kd[2];       //微分项
    float filtered_speed_kd[2];   //滤波后的kd
}Position_PID;

static float LimitFilter(PIDController *pid, float new_speed);
static float MedianFilter(PIDController *pid, float new_speed);
void PID_Init(PIDController *pid, float kp, float ki, float kd, float max_output, float max_integral);
void Motor_Init(Motor_Handle *motor, TIM_HandleTypeDef *PWMtim, uint32_t channel, GPIO_TypeDef *dir_port1, uint16_t dir_pin1, GPIO_TypeDef *dir_port2, uint16_t dir_pin2, 
    TIM_HandleTypeDef *enc_tim, uint32_t enc_channel, GPIO_TypeDef *enc_port1, uint16_t enc_pin1, GPIO_TypeDef *enc_port2, uint16_t enc_pin2);
float PID_compute(PIDController *pid, float raw_speed);
void Motor_Update_Control(Motor_Handle *motor);
void Motor_encoder_callback(Motor_Handle *motor, float interval);
void PID_Callback1(Motor_Handle *motor, TIM_HandleTypeDef *htim, uint32_t channel);
void PID_Callback2(Motor_Handle *motor, TIM_HandleTypeDef *htim, uint32_t channel);
void PID_Callback3(Motor_Handle *motor, TIM_HandleTypeDef *htim, uint32_t channel);
void PID_Callback4(Motor_Handle *motor, TIM_HandleTypeDef *htim, uint32_t channel);
void All_Motors_Init(void);
void wz_PID(PIDController *pid);
float Get_Base_PWM(float target_speed);
float yaw_angle_PD(float yaw_angle);
float Axis_PID_compute(PIDController *pid);




extern Motor_Handle motor1; 
extern Motor_Handle motor2;
extern Motor_Handle motor3;
extern Motor_Handle motor4;
extern PIDController wz; //全局旋转PID控制器
extern PIDController X; //全局X轴PID控制器
extern PIDController Y; //全局Y轴PID控制器
extern float en_x_speed; //全局目标X速度
extern float en_y_speed; //全局目标Y速度
extern Position_PID position_pid; //位置环PID控制器
extern float X_world; //全局世界坐标系X轴位置
extern float Y_world; //全局世界坐标系Y轴位置

#endif 

