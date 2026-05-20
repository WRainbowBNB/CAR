#ifndef __PS2_H__
#define __PS2_H__

#include "main.h"
#include "tim.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "Motor.h"

// ---------------- 按键键值 (PS2标准) ----------------
#define PSB_SELECT      0x0001
#define PSB_L3          0x0002
#define PSB_R3          0x0004
#define PSB_START       0x0008
#define PSB_PAD_UP      0x0010
#define PSB_PAD_RIGHT   0x0020
#define PSB_PAD_DOWN    0x0040
#define PSB_PAD_LEFT    0x0080
#define PSB_L2          0x0100
#define PSB_R2          0x0200
#define PSB_L1          0x0400
#define PSB_R1          0x0800
#define PSB_TRIANGLE    0x1000  // 三角键 -> 遥控模式
#define PSB_CIRCLE      0x2000
#define PSB_CROSS       0x4000  // 叉键 -> 循迹模式
#define PSB_SQUARE      0x8000

// ---------------- 引脚定义 (基于你的图片配置) ----------------

// 1. CS (片选) -> PB13
#define PS2_CS_H()      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define PS2_CS_L()      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)

// 2. CLK (时钟) -> PB12
#define PS2_CLK_H()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define PS2_CLK_L()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

// 3. CMD/MOSI (主机发送) -> PB14
#define PS2_CMD_H()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define PS2_CMD_L()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

// 4. DAT/MISO (主机接收) -> PB15
// 注意：PB14 需要配置为 输入模式 且 上拉 (Pull-Up)
#define PS2_DAT_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)

// ---------------- 函数声明 ----------------
void PS2_Init(void);
void PS2_ReadData(void);
void Data_To_Frame(uint8_t *Data, uint8_t *frameBuf);
void PS2_SetInit(void);
void Data_check(uint8_t *Data);
uint16_t PS2_GetButtons(void);

// 全局变量
extern uint16_t Handkey;
extern uint8_t Data[9];
extern uint8_t last_PS2_time;
extern volatile uint8_t last_PS2_flag;

#endif




