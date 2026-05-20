#include "LineTracking.h"

#define Sub_SMTableS_LEN (sizeof(Sub_SMTableS) / sizeof(Sub_SMTableS[0]))
uint8_t lost_flag = 0;
uint32_t last_lost_time = 0;
uint8_t sensor_status = 0;
//***********子状态机heihei*************/

uint8_t X1, X2, X3, X4, X5;
TraceState current_substate = TRACK_NONE;
TraceEvent subevent = SUBEVENT_NONE;
int trackflag = 0;//经过几次大横杠
int finish_flag1 = 0;//是否完成标志位
int finish_flag2 = 0;//是否完成标志位2
int lock = 0;//是否锁定标志位
int rush_flag = 0;//冲完标志1，没冲完就是0
int system_lock = 0;//系统锁定标志位，1为锁定，0为未锁定
uint8_t yaw_angle = 0;
uint8_t yaw_flag = 0;
uint8_t line_cnt = 0;

void TraceSensor_Read(){
	X1 = HAL_GPIO_ReadPin(X1_GPIO_Port, X1_Pin);
	X2 = HAL_GPIO_ReadPin(X2_GPIO_Port, X2_Pin);
 	X3 = HAL_GPIO_ReadPin(X3_GPIO_Port, X3_Pin);
 	X4 = HAL_GPIO_ReadPin(X4_GPIO_Port, X4_Pin);
	X5 = HAL_GPIO_ReadPin(X5_GPIO_Port, X5_Pin);
	line_cnt = X1 + X2 + X3 + X4 + X5;
}

//处理直线
void handle_line(){
	static float kp = 0.5;
	static float kd = 1.0;
	static float last_error = 0;
	int16_t error = 10 * X1 + 4 * X2  - 4 * X4 - 10 * X5;
	float error_diff = error - last_error;
	last_error = error;
	float vz = kp * error + kd * error_diff;
	//输出控制指令，前进速度固定，旋转速度根据误差调整
	Mecanum_SetSpeed(0, 50, vz);
}

//处理拱桥（直接冲）
void handle_rush(){
	//float yaw_speed = yaw_angle_PD(cur_imu.yaw);
	float yaw_speed = 0;
	//以下是冲桥逻
	static uint32_t stop_start_time = 0;
	if(system_lock == 0){
		system_lock = 1;
		stop_start_time = HAL_GetTick();//开始冲桥计时
	}else{
		uint32_t time_diff = HAL_GetTick() - stop_start_time;
		if(time_diff  <= 800){		//具体冲桥时间记得实测
			Mecanum_SetSpeed(0, 70, 0);
		}else if(time_diff > 800){
			current_substate = TRACK_LINE;
			system_lock = 0;
			yaw_flag = 0;
			rush_flag = 1;
		}
	}
}

//处理断点
void handle_line_break(){
	Mecanum_SetSpeed(0, 30, 1);//这个旋转速度需要实测调整，保证能快速找回黑线
}

//处理十字停车点（1s）
void handle_cross_stop(){
	static uint32_t stop_start_time = 0;
	if(system_lock == 0){
		system_lock = 1;
		stop_start_time = HAL_GetTick();
	}else{
		uint32_t time_diff = HAL_GetTick() - stop_start_time;
		if(time_diff <= 3000){
			Mecanum_SetSpeed(0, 0, 0);	
			Motor_Stop();
		}else if(time_diff > 3000 && time_diff < 3200){
			Mecanum_SetSpeed(0, 15, 0);
			current_substate = TRACK_LINE;
		}else if(time_diff >= 3200){
			Mecanum_SetSpeed(0, 10, 0);
			current_substate = TRACK_LINE;
			system_lock = 0;
			finish_flag1 = 1;
		}
	}
}

//转直角弯先用通电时长，要是没问题再用90°盲转，毕竟JY62买都买了哈哈哈哈哈哈
//处理选择直角弯（根据抽签结果）
void handle_select_turn(){
	static uint32_t stop_start_time = 0;
	if(system_lock == 0){
		system_lock = 1;
		stop_start_time = HAL_GetTick();
	}else{
		uint32_t time_diff = HAL_GetTick() - stop_start_time;
		if(time_diff <= 800){
			Mecanum_SetSpeed(0, 2, -2.5);	//这个看到时候抽签决定正负号
		}
		if(time_diff > 800){
			Mecanum_SetSpeed(0, 2, -2.5);
			current_substate = TRACK_LINE;
			system_lock = 0;
			finish_flag2 = 1;
		}
	}
}

//循迹完毕
void handle_finish_stop(){
	if(system_lock == 0){
		system_lock = 1;
	}else{
		Motor_Stop();
	}
	Mecanum_SetSpeed(0, 0 ,0);
}

//彻底跑偏停车
void handle_stop(){
	Mecanum_SetSpeed(0, 0 ,0);
	//Motor_Stop();
}

//转直角弯先用这个，要是没问题再用JY62,毕竟买都买了哈哈哈哈哈哈
//处理中途直角弯--右转
void handle_right_turn(){
	static uint32_t stop_start_time = 0;
	if(system_lock == 0){
		system_lock = 1;
		stop_start_time = HAL_GetTick();
	}else{
		uint32_t time_diff = HAL_GetTick() - stop_start_time;
		if(time_diff <= 500){
			Mecanum_SetSpeed(0, 0 , -1.75);
		}else if(time_diff > 500){
			current_substate = TRACK_LINE;
			system_lock = 0;
		}
	}
}
//处理中途直角弯--左转
void handle_left_turn(){
	static uint32_t stop_start_time = 0;
	if(system_lock == 0){
		system_lock = 1;
		stop_start_time = HAL_GetTick();
		Mecanum_SetSpeed(0, 0 , 5);	//这个盲转要看具体实测时间
	}else{
		Mecanum_SetSpeed(0, 0 , 5);	
		if(HAL_GetTick() - stop_start_time >= 20){
		current_substate = TRACK_LINE;
		system_lock = 0;
		}
	}
}
//处理全白
static TraceEvent White_Return_Event(){
	//不确定拱桥上黑线删没删，这里先搁着，反正也不影响
	lock = 0;
	// if(rush_flag == 0){
	// 			return SUBEVENT_RUSH;
	// 		}
	
	//记录丢线时间,丢线超过3s停车
	if(lost_flag == 0){
		lost_flag = 1;
		last_lost_time = HAL_GetTick();
	}else if(HAL_GetTick() - last_lost_time >= 3000){
		lost_flag = 0;
		return SUBEVENT_STOP;
	}
	return SUBEVENT_LINE_BREAK;

}
//处理黑线
static TraceEvent Black_Return_Event(){
	static uint32_t last_trackflag_time = 0;
	//防止trackflag过快增加，至少间隔1s
	//一旦检测到黑线，重置丢线标志位
	//每次进入黑线检测，都重置大横杠检测标志位
	//lock = 0;
	// if(rush_flag == 0){
	// 			trackflag = 1;
	// 			rush_flag = 1;
	// 			return SUBEVENT_RUSH;
	// 		}
		lost_flag = 0;
		//中途右直角弯
		if(sensor_status == 0b00111){	//这个数据需要根据实际情况来调整
			return SUBEVENT_RIGHT_TURN;
		}
		else if(sensor_status == 0b11100){	//这个数据需要根据实际情况来调整
			return SUBEVENT_LEFT_TURN;
		}
		else if(sensor_status == 0b11111){	//全黑
			//锁门
			if(lock == 0){
				lock = 1;
				if(trackflag == 0){
					trackflag ++;
					last_trackflag_time = HAL_GetTick();
				}else if(trackflag == 1){
					if(HAL_GetTick() - last_trackflag_time >= 3000){
						trackflag ++;
						last_trackflag_time = HAL_GetTick();
					}
				}else if(trackflag >= 2){
					if(HAL_GetTick() - last_trackflag_time >= 1000){
						trackflag ++;
						last_trackflag_time = HAL_GetTick();
					}
				}	
			}
			//根据trackflag判断当前事件

			switch(trackflag){
				case 1:
					return SUBEVENT_RUSH;//SUBEVENT_RUSH（比赛用这个，开头直接盲冲，再加上航向锁头）
				case 2:
					return SUBEVENT_CROSS_STOP;
				case 3:
					return SUBEVENT_SELECT_TURN;
				case 4:
					trackflag = 0;
					return SUBEVENT_FINISH_STOP;
				default:
					return SUBEVENT_STOP;
			}
		//走直线
		}else{
				// if(rush_flag == 0){
				// 	return SUBEVENT_RUSH;
				// }
				//if(sensor_status == 0b00100 || sensor_status == 0b01000 || sensor_status == 0b00010 || sensor_status == 0b10000 ||sensor_status  == 0b00001 || sensor_status == 0b00101 || sensor_status == 0b10100)
				lock = 0;
			return SUBEVENT_LINE;
		}
}

//将五路循迹状态映射为事件
static TraceEvent Map_Sensor_to_event(){
	//读取传感器状态
	TraceSensor_Read();
	//传感器数据转换成位掩码
	sensor_status = (X1 << 4) | (X2 << 3) | (X3 << 2) | (X4 << 1) | (X5 << 0);
	if(sensor_status == 0b00000){	//全白
		return White_Return_Event();
	}else{
		return Black_Return_Event();
	}
}

//初始化循迹子状态机
void Trace_Init(){
	Motor_Stop();
	current_substate = TRACK_RUSH;
	//current_substate = TRACK_LINE;
	trackflag = 0;
	lost_flag = 0;
	}

//子状态机状态转换表
static const struct Sub_SMTable Sub_SMTableS[] = {
	//当前状态            //事件                 //下一个状态         //回调
	{TRACK_LINE,         SUBEVENT_LINE,            TRACK_LINE,          handle_line},
	{TRACK_LINE,         SUBEVENT_RUSH,            TRACK_RUSH,          handle_rush},
	{TRACK_LINE,         SUBEVENT_LINE_BREAK,      TRACK_LINE_BREAK,    handle_line_break},
	{TRACK_LINE,         SUBEVENT_CROSS_STOP,      TRACK_CROSS_STOP,    handle_cross_stop},
	{TRACK_LINE,         SUBEVENT_SELECT_TURN,     TRACK_SELECT_TURN,   handle_select_turn},
	{TRACK_LINE,         SUBEVENT_RIGHT_TURN,      TRACK_RIGHT_TURN,    handle_right_turn},
	{TRACK_LINE,         SUBEVENT_LEFT_TURN,       TRACK_LEFT_TURN,     handle_left_turn},
	{TRACK_LINE,         SUBEVENT_FINISH_STOP,     TRACK_FINISH_STOP,   handle_finish_stop},
	{TRACK_LINE,         SUBEVENT_STOP,            TRACK_STOP,          handle_stop},
	{TRACK_LINE,         SUBEVENT_EXIT,            EXIT_TRACKING,       Motor_Stop},
	{TRACK_LINE_BREAK,   SUBEVENT_EXIT,            EXIT_TRACKING,       Motor_Stop},
	{TRACK_CROSS_STOP,   SUBEVENT_EXIT,            EXIT_TRACKING,       Motor_Stop},
	{TRACK_SELECT_TURN,  SUBEVENT_EXIT,            EXIT_TRACKING,       Motor_Stop},
	{TRACK_FINISH_STOP,  SUBEVENT_EXIT,            EXIT_TRACKING,       Motor_Stop},
	//{TRACK_CROSS_STOP,   SUBEVENT_LINE,            TRACK_LINE,          handle_line},
	// {TRACK_RIGHT_TURN,   SUBEVENT_LINE,            TRACK_LINE,          handle_line},
	// {TRACK_LEFT_TURN,    SUBEVENT_LINE,            TRACK_LINE,          handle_line},
	{TRACK_LINE_BREAK,   SUBEVENT_LINE,            TRACK_LINE,          handle_line},
	// {TRACK_SELECT_TURN,  SUBEVENT_LINE,            TRACK_LINE,          handle_line},
	{TRACK_RUSH,         SUBEVENT_RIGHT_TURN,      TRACK_RIGHT_TURN,    handle_right_turn},
	//{TRACK_RIGHT_TURN,   SUBEVENT_LINE_BREAK,      TRACK_LINE_BREAK,    handle_line_break},
	{TRACK_LINE_BREAK,   SUBEVENT_CROSS_STOP,      TRACK_CROSS_STOP,    handle_cross_stop},
	{TRACK_CROSS_STOP,   SUBEVENT_SELECT_TURN,     TRACK_SELECT_TURN,   handle_select_turn},
	{TRACK_CROSS_STOP,   SUBEVENT_LINE,            TRACK_LINE,          handle_line},
	{TRACK_NONE,         SUBEVENT_NONE,            TRACK_NONE,          NULL},
	{TRACK_LINE,         SUBEVENT_NONE,            TRACK_LINE,          NULL},
	{TRACK_RUSH,         SUBEVENT_NONE,            TRACK_RUSH,          NULL},
	{TRACK_RUSH,         SUBEVENT_RUSH,            TRACK_RUSH,          handle_rush},
	{TRACK_RUSH,         SUBEVENT_STOP,            TRACK_STOP,          handle_stop},
	{TRACK_LINE_BREAK,   SUBEVENT_LINE_BREAK,      TRACK_LINE_BREAK,    handle_line_break},
	{TRACK_CROSS_STOP,   SUBEVENT_NONE,            TRACK_CROSS_STOP,    NULL},
	{TRACK_SELECT_TURN,  SUBEVENT_NONE,            TRACK_SELECT_TURN,   NULL},
	{TRACK_RIGHT_TURN,   SUBEVENT_NONE,            TRACK_RIGHT_TURN,    NULL},
	{TRACK_LEFT_TURN,    SUBEVENT_NONE,            TRACK_LEFT_TURN,     NULL},
	{TRACK_FINISH_STOP,  SUBEVENT_NONE,            TRACK_FINISH_STOP,   NULL},
	{TRACK_FINISH_STOP,  SUBEVENT_STOP,            TRACK_STOP,          handle_stop},
	{TRACK_FINISH_STOP,  SUBEVENT_LINE,            TRACK_FINISH_STOP,   handle_finish_stop},
	{TRACK_FINISH_STOP,  SUBEVENT_LINE_BREAK,      TRACK_FINISH_STOP,   handle_finish_stop},
	{TRACK_STOP,         SUBEVENT_NONE,            TRACK_STOP,          NULL},
	{TRACK_NONE,         SUBEVENT_RUSH,            TRACK_RUSH,          handle_rush},
	{TRACK_NONE,         SUBEVENT_LINE,            TRACK_LINE,          handle_line},
	{TRACK_LINE_BREAK,   SUBEVENT_STOP,            TRACK_STOP,          handle_stop},
	{TRACK_RUSH,   		 SUBEVENT_CROSS_STOP,      TRACK_CROSS_STOP,    handle_cross_stop},	
	{TRACK_RIGHT_TURN,   SUBEVENT_LINE,            TRACK_LINE,          handle_line},	
	{TRACK_LEFT_TURN,    SUBEVENT_LINE,            TRACK_LINE,          handle_line},
	{TRACK_SELECT_TURN,  SUBEVENT_LINE,            TRACK_LINE,          handle_line},
};

//子状态机状态转换函数
void Sub_State_Transmission(){
    for (int i = 0; i < Sub_SMTableS_LEN; i++){
        if(current_substate == Sub_SMTableS[i].current_substate && subevent == Sub_SMTableS[i].subevent){
            current_substate = Sub_SMTableS[i].new_substate;
            break;
        }
    };
}

void Sub_State_Run(){
        switch(current_substate){
				case TRACK_LINE:
					handle_line();
					break;
				case TRACK_RUSH:
					handle_rush();
					//handle_line();
					break;
				case TRACK_LINE_BREAK:
					handle_line_break();
					break;
				case TRACK_CROSS_STOP:
					handle_cross_stop();
					break;
				case TRACK_SELECT_TURN:
					handle_select_turn();
					break;
				case TRACK_RIGHT_TURN:
					handle_right_turn();
					//handle_line();
					break;
				case TRACK_LEFT_TURN:
					//handle_left_turn();
					handle_line();
					break;
				case TRACK_FINISH_STOP:
					handle_finish_stop();
					break;
				case TRACK_STOP:
					handle_stop();
					break;
				case EXIT_TRACKING:
					Motor_Stop();
					break;
				default:
					handle_stop();
		}
}

//判断是否完成循迹
int Trace_Run(){
	subevent = Map_Sensor_to_event();
	//Sub_State_Transmission();
	if(rush_flag == 0){
		current_substate = TRACK_RUSH;
		handle_rush();
	}
	if(trackflag == 2 && finish_flag1 == 0){
		current_substate = TRACK_CROSS_STOP;
		handle_cross_stop();
	}else if(trackflag == 3 && finish_flag2 == 0){
		current_substate = TRACK_SELECT_TURN;
		handle_select_turn();
	}else if(trackflag == 4){
		current_substate = TRACK_FINISH_STOP;
		handle_finish_stop();
	}
	if(system_lock == 0){	
		Sub_State_Transmission();
	}
	Sub_State_Run();
	if(current_substate == EXIT_TRACKING){
		return 1;
	}
	else{
		return 0;
	}
}

