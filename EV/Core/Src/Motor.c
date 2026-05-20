#include "Motor.h"

float en_fl_speed = 0.0f;
float en_fr_speed = 0.0f;
float en_rl_speed = 0.0f;
float en_rr_speed = 0.0f;

void MOTOR_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void Mecanum_SetSpeed(float vx, float vy, float vz){
	target_vx = vx; //传入的为期望速度，更新全局目标速度供PID使用
	target_vy = vy; //传入的为期望速度，更新全局目标速度供PID使用
	target_vz = vz; //传入的为期望旋转速度，更新全局目标旋转速度供PID使用
}

void Mecanum_Solver(void){
	//float target_pid_vz = wz.output; //更新全局目标旋转速度为pid后的速度(到了用这个)
	float target_pid_vz = target_vz; //直接使用全局目标旋转速度进行解算(imu没到捏)
	// 麦轮逆运动学解算
	float fl_speed = (target_vy + target_vx - target_pid_vz * k_fl);  //左前轮
	float fr_speed = (target_vy - target_vx + target_pid_vz * k_fr);  //右前轮
	float rl_speed = (target_vy - target_vx - target_pid_vz * k_rl);  //左后轮
	float rr_speed = (target_vy + target_vx + target_pid_vz * k_rr);  //右后轮
	
	//单位：cm/s
	motor1.pid.target_speed = fl_speed; //更新电机1目标速度供PID使用
	motor2.pid.target_speed = fr_speed; //更新电机2目标速度供PID使用
	motor3.pid.target_speed = rl_speed; //更新电机3目标速度供PID使用
	motor4.pid.target_speed = rr_speed; //更新电机4目标速度供PID使用
		
	//转PWM并限制范围
	int16_t fl_pwm = LIMIT_PWM(fl_speed * PWM_SCALE);	
	int16_t fr_pwm = LIMIT_PWM(fr_speed * PWM_SCALE);
	int16_t rl_pwm = LIMIT_PWM(rl_speed * PWM_SCALE);
	int16_t rr_pwm = LIMIT_PWM(rr_speed * PWM_SCALE);
	
	//输出PWM
	Motor1_SetSpeed(fl_pwm);
	Motor2_SetSpeed(fr_pwm);
	Motor3_SetSpeed(rl_pwm);
	Motor4_SetSpeed(rr_pwm);

}

void Mecanum_ReadSpeed(void){
	en_fl_speed = motor1.pid.current_speed; //左前轮
    en_fr_speed = motor2.pid.current_speed; //右前轮
    en_rl_speed = motor3.pid.current_speed; //左后轮
    en_rr_speed = motor4.pid.current_speed; //右后轮
}
void Motor1_SetSpeed(int16_t Speed)
{
	if(Speed >= 0)
	{		
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Speed);
	}
	else{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -Speed);
	}
}	

void Motor2_SetSpeed(int16_t Speed)
{
	if(Speed >= 0)
	{		
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Speed);
	}
	else{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -Speed);
	}
}
void Motor3_SetSpeed(int16_t Speed)
{
	if(Speed >= 0)
	{		
		HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Speed);
	}
	else{
		HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -Speed);
	}
}
void Motor4_SetSpeed(int16_t Speed)
{
	if(Speed >= 0)
	{		
		HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Speed);
	}
	else{
		HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -Speed);
	}
}
void Motor_Stop(void){
	Motor1_SetSpeed(0);
	Motor2_SetSpeed(0);
	Motor3_SetSpeed(0);
	Motor4_SetSpeed(0);
}