/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SM.h"
#include "PS2.h"
#include "Motor.h"
#include "LineTracking.h"
#include "oled.h"
#include "font.h"
#include "Controller.h"
#include "time_slice.h"
#include "stdio.h"
#include "encoder.h"
#include "imu.h"
#include "test.h"
#include "string.h"
#include "yaw_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
time_slice PS2_time_slice;
time_slice OLED_time_slice;
time_slice UART_PS2_time_slice;
uint32_t tim12_tick = 0; // 用于10ms定时器
uint8_t imu_data[11]; // IMU数据缓冲区

uint32_t real_pwm_motor1 = 0; // 实时PWM值1
uint32_t real_pwm_motor2 = 0; // 实时PWM值2
uint32_t real_pwm_motor3 = 0; // 实时PWM值3
uint32_t real_pwm_motor4 = 0; // 实时PWM值4

char test[50] = ""; //调试串口数据缓冲区
uint8_t last_send_time = 0; // 串口上次发送时间
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C3_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_IWDG_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	// HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	// HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	// HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  // HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40);
  // HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 40);
  // HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_SET);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 40);
  HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 40);
  while(1){

  }
  PS2_Init();
	MOTOR_Init();
  Robotic_Arm_Init();
  OLED_Init(); 
  OLED_NewFrame();
  OLED_ShowFrame();
  time_slice_Init(&PS2_time_slice, 50, PS2_collect_callback);
  time_slice_Init(&OLED_time_slice, 300, OLED_callback);
  All_Motors_Init(); // 初始化所有电机（包含PID参数初始化）
  HAL_TIM_Base_Start_IT(&htim12); //10ms做一次pid
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4); //编码器1输入捕获
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE); //编码器1溢出中断
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); //编码器2输入捕获
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE); //编码器2溢出中断
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); //编码器3输入捕获
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE); //编码器3溢出中断
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1); //编码器4输入捕获
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE); //编码器4溢出中断
  //这个是调pid的时候的串口接收函数
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)rx_buf, sizeof(rx_buf)-1);
  //HAL_UART_Receive_IT(&huart1, (uint8_t*)test, 1);
  HAL_UART_Receive_DMA(&huart4, (uint8_t*)rx_buf, sizeof(rx_buf));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)rx_buf1, sizeof(rx_buf1));
  //HAL_UART_Receive_DMA(&huart3, test, (uint8_t*)rx_buf1, sizeof(rx_buf1)-1);
  
  PID_Init(&wz, 0.5f, 0.01f, 0.0f, 100.0f, 100.0f); // 初始化yaw轴PID控制
  //Motor2_SetSpeed(40);
  //Motor3_SetSpeed(40);
  // Motor4_SetSpeed(40);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    //handle_right_turn();
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 150);
    //__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 100);  //具体数值待测
     //HAL_UART_Transmit(&huart3, (uint8_t *)"!", 1, 10);
     //Motor3_SetSpeed(30);
     //Motor2_SetSpeed(30);
     //Motor1_SetSpeed(30);
    // HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 40);
   //Motor4_SetSpeed(30);
    // //Motor_Stop();
     real_pwm_motor1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
     real_pwm_motor2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);  
     real_pwm_motor3 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
     real_pwm_motor4 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    // sprintf(test, "wz: %.2f, yaw: %.2f\n", cur_imu.wz, cur_imu.yaw);
    //sprintf(test, "pules: %d\n, interval: %.2f\n", motor1.pulse_cnt, motor1.interval);
    //sprintf(test, "E: %.2f, T: %.2f, C: %.2f\n", en_x_speed, target_vx, X.current_speed);
    //sprintf(test, "E: %.2f, T: %.2f, C: %.2f\n", en_y_speed, target_vy, Y.current_speed);
    //sprintf(test, "1: %u, 2: %u\n", claw_status, test_status);
    //sprintf(test, "%u\n", sensor_status);
    //sprintf(test, "[1]: %u, [2]: %u\n", Data[7], Data[8]);
    // sprintf(test, "X: %f, Y:%f, Z:%f\n", target_vx, target_vy, target_vz);
    //sprintf(test, "flag: %u\n", line_cnt);
    //sprintf(test, "trackflag: %d\n", trackflag);
    //sprintf(test, "kp: %.2f, ki: %.2f, kd: %.2f\n", motor4.pid.kp, motor4.pid.ki, motor4.pid.kd);
    //sprintf(test, "cur_direction: %d\n", motor4.cur_direction);
    //sprintf(test, "1: %u, 2: %u, 3: %u, 4: %u\n", real_pwm_motor1, real_pwm_motor2, real_pwm_motor3, real_pwm_motor4);
    //sprintf(test, "pules: %d\n, interval: %.2f\n", motor4.pulse_cnt, motor4.interval);
    //sprintf(test, "vx: %.2f, vy: %.2f, vz: %.2f\n", target_vx, target_vy, target_vz);
     //sprintf(test, "1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f\n", motor1.pid.current_speed, motor2.pid.current_speed, motor3.pid.current_speed, motor4.pid.current_speed);
    sprintf(test, "1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f\n", motor1.pid.output, motor2.pid.output, motor3.pid.output, motor4.pid.output);
    //HAL_UART_Transmit(&huart3, (uint8_t *)test, strlen(test), 100);
    HAL_UART_Transmit(&huart3, (uint8_t *)"!", 1, 10);
    // --- 1.PS2数据采集 ---
    time_slice_run(&PS2_time_slice);
    State_Run();  //状态运行                                                                                                           
   
    HAL_IWDG_Refresh(&hiwdg); // 莫得问题，喂狗
  
    // //****调试点1.串口调试（PS2)
    //PS2_Test();

    // //--- 2.IMU数据处理
		Process_IMU_Data(&Rx_Buffer);

    // //---  3.OLED 显示 ---
		 time_slice_run(&OLED_time_slice);
    // --- 这个是调pid的时候的串口发送函数，写进时间片是为了避免dma传重复数据 ---
    if(HAL_GetTick() - last_send_time > 40){
     //这俩选一个来调
     if(test_status == 1){
      //  VOFA_Speed_Test();
       VOFA_Speed_Test1();
        //VOFA_Speed_Test2();
        //VOFA_Yaw_Test();
        //VOFA_JY62_Test();
        //VOFA_KALMAN_Test();
     } 
      last_send_time = HAL_GetTick();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//处理IMU数据，用串口4
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == UART4){
      Process_IMU_Data(&Rx_Buffer);
      // IMU_Update(rx_buf);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
     // HAL_UART_Transmit_IT(&huart3, (uint8_t *)"!", 1);

    }
    if(huart->Instance == USART1){
      //这里是接收处理函数，两个pid选一个调
			 	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        //HAL_UART_Transmit_DMA(&huart3, (uint8_t*)rx_buf1, sizeof(rx_buf1));
        //HAL_UART_Transmit(&huart3, (uint8_t *)"!", 1, 10);
			  Process_Bluetooth_Data(&Rx_Buffer1);
    }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart->Instance == UART4){
//     Process_IMU_Data(&Rx_Buffer);
//     // HAL_UART_Receive_IT(&huart3, (uint8_t*)test, 1);
//   }
// }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //定时器溢出回调，用于编码器时间间隔计算
  if(htim->Instance == TIM5)
  {
    __disable_irq();
    motor1.overflow_cnt++;
    __enable_irq();
  }
  else if(htim->Instance == TIM3)
  {
    __disable_irq();
    motor2.overflow_cnt++;
    __enable_irq();
  }
  else if(htim->Instance == TIM4)
  {
    __disable_irq();
    motor3.overflow_cnt++;
    __enable_irq();
  }
  else if(htim->Instance == TIM8)
  {
    __disable_irq();
    motor4.overflow_cnt++;
    __enable_irq();
  }

  //10ms中断一次，进行PID计算
  if(htim->Instance == TIM12) //10ms中断一次，进行PID计算
  {
    tim12_tick++;
    if(tim12_tick >= 5){
      tim12_tick = 0;
      //Kalman_yaw = Kalman_Filter(encoder_yaw_speed());
      //wz_PID(&wz); //计算旋转PID输出
      //Y_PID(&Y);
      //X_PID(&X);
      Mecanum_Solver(); // 麦轮解算，计算各轮目标速度
      // Motor_Update_Control(&motor1);
      // Motor_Update_Control(&motor2);
      // Motor_Update_Control(&motor3);
      // Motor_Update_Control(&motor4);

      // position_pid.angle += PID_T * yaw_filter.w_k ;

    }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  if(htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
    PID_Callback1(&motor1, htim, TIM_CHANNEL_4);
  }

  if(htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
    PID_Callback2(&motor2, htim, TIM_CHANNEL_1);
  }

  if(htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
    PID_Callback3(&motor3, htim, TIM_CHANNEL_1);
  }

  if(htim->Instance == TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
    PID_Callback4(&motor4, htim, TIM_CHANNEL_1);
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
