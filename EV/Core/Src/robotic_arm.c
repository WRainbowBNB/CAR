#include "robotic_arm.h"

#define ARM_K 0.1f  //机械臂系数，根据实际情况调整
void Robotic_Arm_Init(){
	//初始化机械臂
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);	//爪子
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);	//机械臂
}                     

void claw_open(){
	//打开爪子
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 70);  //具体数值待测
}

void claw_close(){
	//关闭爪子
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 170);	//具体数值待测
}

//用右摇杆Y轴模拟量控制机械臂上下旋
//效果有待实测
void arm_controller(){
	static uint32_t last_time = 0;
	static uint32_t flag = 0;
	if(flag == 0){
		flag = 1;
		last_time = HAL_GetTick();

	}else{
		if(HAL_GetTick() - last_time >= 50) 
		{
			int16_t RY = 128 - Data[6];
			static float angle = 150.0f;
			float delta_angle = 0.0f;
			//如果摇杆位于死区，则不控制角度增量
			if(abs(RY) < 20) delta_angle = 0;
			else delta_angle = RY * ARM_K;	//此系数需要根据实际情况调整
			angle += delta_angle; 
			//加个限幅，避免角度超出范围烧舵机
			if(angle > 250.0f) angle = 250.0f;
			else if(angle < 100.0f) angle = 100.0f;
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, angle);
			flag = 0;
		}
		
	}
	
}