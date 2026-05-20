/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM9_Steering1_Pin GPIO_PIN_5
#define TIM9_Steering1_GPIO_Port GPIOE
#define TIM9_Steering2_Pin GPIO_PIN_6
#define TIM9_Steering2_GPIO_Port GPIOE
#define Encoder1_B_Pin GPIO_PIN_2
#define Encoder1_B_GPIO_Port GPIOA
#define Encoder1_A_Pin GPIO_PIN_3
#define Encoder1_A_GPIO_Port GPIOA
#define lsmotor_Pin GPIO_PIN_5
#define lsmotor_GPIO_Port GPIOA
#define Encoder2_A_Pin GPIO_PIN_6
#define Encoder2_A_GPIO_Port GPIOA
#define Encoder2_B_Pin GPIO_PIN_7
#define Encoder2_B_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_5
#define AIN1_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_0
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_1
#define IN2_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_7
#define AIN2_GPIO_Port GPIOE
#define BIN2_Pin GPIO_PIN_8
#define BIN2_GPIO_Port GPIOE
#define PWMA_Pin GPIO_PIN_9
#define PWMA_GPIO_Port GPIOE
#define BIN1_Pin GPIO_PIN_10
#define BIN1_GPIO_Port GPIOE
#define PWMB_Pin GPIO_PIN_11
#define PWMB_GPIO_Port GPIOE
#define CIN1_Pin GPIO_PIN_12
#define CIN1_GPIO_Port GPIOE
#define PWMC_Pin GPIO_PIN_13
#define PWMC_GPIO_Port GPIOE
#define PWMD_Pin GPIO_PIN_14
#define PWMD_GPIO_Port GPIOE
#define CIN2_Pin GPIO_PIN_15
#define CIN2_GPIO_Port GPIOE
#define DIN2_Pin GPIO_PIN_10
#define DIN2_GPIO_Port GPIOB
#define DIN1_Pin GPIO_PIN_11
#define DIN1_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_12
#define CLK_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_13
#define CS_GPIO_Port GPIOB
#define CMD_Pin GPIO_PIN_14
#define CMD_GPIO_Port GPIOB
#define DAT_Pin GPIO_PIN_15
#define DAT_GPIO_Port GPIOB
#define Encoder3_A_Pin GPIO_PIN_12
#define Encoder3_A_GPIO_Port GPIOD
#define Encoder3_B_Pin GPIO_PIN_13
#define Encoder3_B_GPIO_Port GPIOD
#define Encoder4_A_Pin GPIO_PIN_6
#define Encoder4_A_GPIO_Port GPIOC
#define Encoder4_B_Pin GPIO_PIN_7
#define Encoder4_B_GPIO_Port GPIOC
#define X1_Pin GPIO_PIN_3
#define X1_GPIO_Port GPIOD
#define X2_Pin GPIO_PIN_4
#define X2_GPIO_Port GPIOD
#define X3_Pin GPIO_PIN_5
#define X3_GPIO_Port GPIOD
#define X4_Pin GPIO_PIN_6
#define X4_GPIO_Port GPIOD
#define X5_Pin GPIO_PIN_7
#define X5_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
typedef struct 
{
  /* data */
}Encoder;

extern UART_HandleTypeDef huart1;
extern char message[50];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
