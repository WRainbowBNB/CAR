#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"
#include "stdio.h"
#include "string.h"

typedef struct{
    float ax, ay, az; //加速度
    float wx, wy, wz; //角速度
    float yaw, pitch, roll; //角度
}IMU_Data;

typedef struct{
	uint8_t *buf;
	uint8_t buf_size;
	uint8_t old_pos;
    uint16_t line_pos; 		// 当前行数据位置
	uint8_t line_temp[128]; // 用于存储一行数据
} RXBUFFER_U;

void IMU_Update(uint8_t *data);
void Process_IMU_Data(RXBUFFER_U *rxbuffer);

extern IMU_Data cur_imu; //当前IMU数据
extern RXBUFFER_U Rx_Buffer;
extern uint8_t rx_buf[64];

#endif 

