#ifndef __TEST_H__
#define __TEST_H__

#include "main.h"
#include "PS2.h"
#include "usart.h"
#include "encoder.h"
#include "yaw_filter.h"

typedef struct{
	char *buf;
	uint8_t buf_size;
	uint8_t old_pos;
	uint16_t line_pos; 		// 当前行数据位置
	uint8_t line_temp[128]; // 用于存储一行数据
} RXBUFFER;

#pragma pack(1)
typedef struct{
	float target_speed;
	float cur_speed;
	float error;
	uint8_t tail[4];	//固定尾帧
} VOFA_JustFloat;
#pragma pack()

#pragma pack(1)
typedef struct{
	float current_speed;
	uint8_t tail[4];	//固定尾帧
} VOFA_JustFloat1;
#pragma pack()

#pragma pack(1)
typedef struct{
	float speed1;
	float speed2;
	float speed3;
	float speed4;
	uint8_t tail[4];	//固定尾帧
} VOFA_JustFloat2;
#pragma pack()

//PS2测试函数
void PS2_Test();
//PS2测试函数
void PS2_Test();
//FireWater通信协议，向上位机发送相关数据
void VOFA_Speed_Test(void);
void VOFA_Yaw_Test(void);
void VOFA_JY62_Test(void);
void VOFA_KALMAN_Test(void);
//内环速度环的pid控制，配合VOFA+使用
void Speed_PID(char *line_temp);
//外环yaw轴pid控制，配合VOFA+使用
void Yaw_PID(char *line_temp);
void Process_Bluetooth_Data(RXBUFFER *rxbuffer);
void VOFA_X_Test(void);
void VOFA_Y_Test(void);
void VOFA_Speed_Test1(void);
void VOFA_Speed_Test2(void);

extern RXBUFFER Rx_Buffer1;
extern RXBUFFER Rx_Buffer2;

extern char rx_buf1[64];
extern char rx_buf2[64];
extern char rx_buf3[64];

#endif 


	
		 

