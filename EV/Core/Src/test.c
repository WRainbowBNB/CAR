#include "test.h"

#define FRAME_HEADER 0xAA
#define FRAME_TAILER 0xFF
#define MAX_FRAME_LENGTH 11

char rx_buf1[64];
char rx_buf2[64];


uint8_t old_pos = 0;  //上次处理到的位置
uint8_t line_temp[128];  // 用于存储一行数据
uint16_t line_pos = 0;  // 当前行数据位置

RXBUFFER Rx_Buffer1 = {
	.buf = rx_buf1,
	.buf_size = sizeof(rx_buf1),
  .old_pos = 0,
  .line_pos = 0,
  .line_temp = {0},
};

RXBUFFER Rx_Buffer2 = {
	.buf = rx_buf2,
	.buf_size = sizeof(rx_buf2),
	.old_pos = 0,
  .line_pos = 0,
  .line_temp = {0},
};

//这里是PS2测试函数
void PS2_Test(){
  //c.串口打印（检测用的），这个东西放main的时间片轮询里
  uint8_t frameBuf[MAX_FRAME_LENGTH]; //串口打印用
  uint8_t is_tx_busy = 0;
  static uint32_t last_time = 0; // 用于控制打印频率
  if (HAL_GetTick() - last_time > 200) 
  {
    last_time = HAL_GetTick(); // 更新时间
    Data_To_Frame(Data, frameBuf);
    //串口3看PS2数据是否正常发送
    HAL_UART_Transmit(&huart3, frameBuf, 11, 100);
  }
}

// FireWater数据协议  换行结尾  /n或/r/n  逗号分隔通道
//指定三个通道，注意发数据用的是串口1，收用3，记得测试完PS2把上面的注释掉
//修改 VOFA_Speed_Test 以便观察目标值变化
//这个是纯看,看实时变化用的
void VOFA_Speed_Test(void)   
{
  if(huart1.gState == HAL_UART_STATE_READY){
    static VOFA_JustFloat justFloat = {
      .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    justFloat.target_speed = motor1.pid.target_speed;
    justFloat.cur_speed = motor1.pid.current_speed;
    justFloat.error = motor1.pid.error;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&justFloat, sizeof(VOFA_JustFloat));
    // // 将 150.0 改为实际的目标变量
    // static char test[50] = ""; //调试串口数据缓冲区
    // sprintf(test, "%.2f,%.2f,%.2f\n", motor1.pid.current_speed, motor1.pid.target_speed, motor1.pid.error);
    // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)test, strlen(test));
  }
}

void VOFA_Speed_Test1(void)   
{
  if(huart1.gState == HAL_UART_STATE_READY){
    static VOFA_JustFloat1 justFloat = {
      .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    justFloat.current_speed = motor4.pid.current_speed;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&justFloat, sizeof(VOFA_JustFloat1));
    // // 将 150.0 改为实际的目标变量
    // static char test[50] = ""; //调试串口数据缓冲区
    // sprintf(test, "%.2f\n", motor3.pid.target_speed);
    // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)test, strlen(test));
  }
}
void VOFA_Speed_Test2(void)   
{
  if(huart1.gState == HAL_UART_STATE_READY){
    static VOFA_JustFloat2 justFloat = {
      .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    justFloat.speed1 = motor1.pid.current_speed;
    justFloat.speed2 = motor2.pid.current_speed;
    justFloat.speed3 = motor3.pid.current_speed;
    justFloat.speed4 = motor4.pid.current_speed;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&justFloat, sizeof(VOFA_JustFloat2));
    // // 将 150.0 改为实际的目标变量
    // static char test[50] = ""; //调试串口数据缓冲区
    // sprintf(test, "%.2f\n", motor3.pid.target_speed);
    // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)test, strlen(test));
  }
}

void VOFA_Yaw_Test(void)   
{
  if(huart1.gState == HAL_UART_STATE_READY){
    static VOFA_JustFloat justFloat = {
      .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    justFloat.target_speed = wz.target_speed;
    justFloat.cur_speed = wz.current_speed;
    justFloat.error = wz.error;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&justFloat, sizeof(VOFA_JustFloat));
    // // 将 150.0 改为实际的目标变量
    // static char test[50] = ""; //调试串口数据缓冲区
    // sprintf(test, "%.2f,%.2f,%.2f\n", wz.current_speed, wz.target_speed, wz.error);
    // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)test, strlen(test));
  }
}

void VOFA_X_Test(void)   
{
  if(huart1.gState == HAL_UART_STATE_READY){
    static VOFA_JustFloat justFloat = {
      .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    justFloat.target_speed = target_vx;
    justFloat.cur_speed = X.current_speed;
    justFloat.error = en_x_speed;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&justFloat, sizeof(VOFA_JustFloat));
  }
}

void VOFA_Y_Test(void)   
{
  if(huart1.gState == HAL_UART_STATE_READY){
    static VOFA_JustFloat justFloat = {
      .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    justFloat.target_speed = target_vy;
    justFloat.cur_speed = Y.current_speed;
    justFloat.error = en_y_speed;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&justFloat, sizeof(VOFA_JustFloat));
  }
}

//纯粹是为了检查这个模块好不好使
void VOFA_JY62_Test(void)   
{
  if(huart1.gState == HAL_UART_STATE_READY){
    static VOFA_JustFloat justFloat = {
      .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    justFloat.target_speed = yaw_filter.w_k;
    justFloat.cur_speed = cur_imu.wz;
    justFloat.error = encoder_yaw_speed();
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&justFloat, sizeof(VOFA_JustFloat));
    // // 将 150.0 改为实际的目标变量
    // static char test[50] = ""; //调试串口数据缓冲区
    // sprintf(test, "%.2f,%.2f,%.2f\n", cur_imu.roll, cur_imu.pitch, cur_imu.yaw);
    // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)test, strlen(test));
  }
}

void VOFA_KALMAN_Test(void)   
{
  if(huart1.gState == HAL_UART_STATE_READY){
    float en_yaw_speed = encoder_yaw_speed();
    static VOFA_JustFloat justFloat = {
      .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    justFloat.target_speed = yaw_filter.w_pre;
    justFloat.cur_speed = yaw_filter.w_k;
    justFloat.error = en_yaw_speed;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&justFloat, sizeof(VOFA_JustFloat));
    // // 将 150.0 改为实际的目标变量
    // static char test[50] = ""; //调试串口数据缓冲区
    // sprintf(test, "%.2f,%.2f,%.2f\n", yaw_filter.w_pre, yaw_filter.w_k, yaw_filter.error1);
    // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)test, strlen(test));
  }
}

//******调试点4 yaw轴pid
void Yaw_PID(char *line_temp)
{
    // // 安全处理缓冲区：防止溢出+清空末尾乱码
    //    if (Rx_Buffer2.line_pos > 0 && Rx_Buffer2.line_pos < sizeof(rx_buf1)) {
    //         line_temp[Rx_Buffer2.line_pos - 1] = '\0'; // 字符串结尾
    //    } else {
    //         line_temp[0] = '\0'; // 无效数据清空
    //    }
    float p, i, d, s;
    // 兼容VOFA+两种格式（有/无空格），指令结尾支持\n
    // 解析PID指令：匹配 PID:1.2,3.4,5.6 或 PID: 1.2, 3.4, 5.6
    if (sscanf(line_temp, "KP:%f", &p) == 1 || 
        sscanf(line_temp, "KP: %f", &p) == 1){
        wz.kp = p;
        wz.integral_sum  = 0; // 修改PID后清空积分，防止震荡
    }
    else if (sscanf(line_temp, "KI:%f", &i) == 1 || 
            sscanf(line_temp, "KI: %f", &i) == 1)
    {
            wz.ki = i;
            wz.integral_sum = 0; // 修改PID后清空积分，防止震荡
    }
    else if (sscanf(line_temp, "KD:%f", &d) == 1 || 
            sscanf(line_temp, "KD: %f", &d) == 1)
    {
        wz.kd = d;
        wz.integral_sum = 0; // 修改PID后清空积分，防止震荡
    }
}

// UART DMA空闲接收回调（解析VOFA+指令）
//这个应该是中转串口和上位机之间的通信
void Speed_PID(char *line_temp)
{
        
  float p, i, d, s;
  // 2. 兼容VOFA+两种格式（有/无空格），指令结尾支持\n
  // 解析PID指令：匹配 PID:1.2,3.4,5.6 或 PID: 1.2, 3.4, 5.6
  if (sscanf(line_temp, "KP:%f", &p) == 1)
  {
    motor1.pid.kp = motor2.pid.kp = motor3.pid.kp = motor4.pid.kp = p;
    motor1.pid.integral_sum = motor2.pid.integral_sum = motor3.pid.integral_sum = motor4.pid.integral_sum = 0; // 修改PID后清空积分，防止震荡
  }
  else if (sscanf(line_temp, "KI:%f", &i) == 1)
  {
    motor1.pid.ki = motor2.pid.ki = motor3.pid.ki = motor4.pid.ki = i;
    motor1.pid.integral_sum = motor2.pid.integral_sum = motor3.pid.integral_sum = motor4.pid.integral_sum = 0; // 修改PID后清空积分，防止震荡
  }
  else if (sscanf(line_temp, "KD:%f", &d) == 1)
  {
    motor1.pid.kd = motor2.pid.kd = motor3.pid.kd = motor4.pid.kd = d; 
    motor1.pid.integral_sum = motor2.pid.integral_sum = motor3.pid.integral_sum = motor4.pid.integral_sum = 0; // 修改PID后清空积分，防止震荡
  }
  // 解析转速指令：匹配 SPEED:130 或 SPEED: 130
  else if (sscanf(line_temp, "SPEED:%f", &s) == 1)
  {
    motor1.pid.target_speed = motor2.pid.target_speed = motor3.pid.target_speed = motor4.pid.target_speed = s; // 直接设置目标速度，PID计算时会自动渐变
    motor1.pid.integral_sum = motor2.pid.integral_sum = motor3.pid.integral_sum = motor4.pid.integral_sum = 0; // 修改PID后清空积分，防止震荡
  }
 }

//  //字符串可以写成数据帧？？？
//  //关于data_flag：1-解析速度指令，2-解析yaw指令，其他数则当普通接收队列使用
// //  void Process_Bluetooth_Data(RXBUFFER *rxbuffer, int data_flag){
// //   //获取DMA还需搬运字节数
// //   uint16_t remaining_space = __HAL_DMA_GET_COUNTER(rxbuffer -> uart -> hdmarx);
// //   //计算DMA搬运到的数组下标,取余避免越界
// //   uint8_t dma_write_ptr = (rxbuffer -> buf_size - remaining_space) % rxbuffer -> buf_size;
// //   //写指针变化，说明有新数据到达
// //   if(dma_write_ptr != rxbuffer -> old_pos){
// //     // 处理新数据
// //     while(rxbuffer -> old_pos != dma_write_ptr){
// //       uint8_t ch = rxbuffer -> buf[rxbuffer -> old_pos];
// //       //存字节
// //       rxbuffer -> line_temp[rxbuffer -> line_pos++] = ch;
// //       //利用通信协议，判断什么时候该发
// //         if(ch == '\n'){
// //             rxbuffer -> line_temp[rxbuffer -> line_pos - 1] = '\0'; // 字符串结尾
// //             // if(data_flag == 1) Speed_PID((char *)rxbuffer -> line_temp);
// //             // else if(data_flag == 2) Yaw_PID((char *)rxbuffer -> line_temp);
// //             //这个是看数据接收到了没有的
// //             rxbuffer->line_temp[rxbuffer->line_pos - 1] = '\n';
// //             HAL_UART_Transmit(&huart3, rxbuffer -> line_temp, rxbuffer -> line_pos, 10);
// //             rxbuffer -> line_pos = 0; // 重置行数据位置
// //         }
// //       //指针往后面挪，挪到末尾自动回到0
// //       rxbuffer -> old_pos ++;
// //       if(rxbuffer -> old_pos >= rxbuffer -> buf_size)  rxbuffer -> old_pos = 0; // 循环利用缓冲区
// //       if(rxbuffer -> line_pos >= 60) rxbuffer -> line_pos = 0; // 防止行数据溢出
// //     }
// //   }
// // }

void Process_Bluetooth_Data(RXBUFFER *rxbuffer){
  //获取DMA还需搬运字节数
  uint16_t remaining_space = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
  //计算DMA搬运到的数组下标,取余避免越界
  uint8_t dma_write_ptr = (rxbuffer -> buf_size - remaining_space) % rxbuffer -> buf_size;
  //写指针变化，说明有新数据到达
  if(dma_write_ptr != rxbuffer -> old_pos){
    // 处理新数据
    while(rxbuffer -> old_pos != dma_write_ptr){
      uint8_t ch = rxbuffer -> buf[rxbuffer -> old_pos];
      //存字节
      rxbuffer -> line_temp[rxbuffer -> line_pos++] = ch;
      //利用通信协议，判断什么时候该发
        if(ch == '\n'){
          rxbuffer -> line_temp[rxbuffer -> line_pos - 1] = '\0'; // 字符串结尾 
          Speed_PID((char *)rxbuffer -> line_temp);
          //HAL_UART_Transmit_IT(&huart3, rxbuffer -> line_temp, rxbuffer -> line_pos);
          rxbuffer -> line_pos = 0; // 重置行数据位置
        }
      //指针往后面挪，挪到末尾自动回到0
      rxbuffer -> old_pos ++;
      if(rxbuffer -> old_pos >= rxbuffer -> buf_size)  rxbuffer -> old_pos = 0; // 循环利用缓冲区
      if(rxbuffer -> line_pos >= 60) rxbuffer -> line_pos = 0; // 防止行数据溢出
    }
  }
}

//  void Process_Bluetooth_Data(RXBUFFER *rxbuffer){
//   //获取DMA还需搬运字节数
//   uint16_t remaining_space = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
//   //计算DMA搬运到的数组下标,取余避免越界
//   uint8_t dma_write_ptr = (rxbuffer -> buf_size - remaining_space) % rxbuffer -> buf_size;
//   //写指针变化，说明有新数据到达
//   if(dma_write_ptr != rxbuffer -> old_pos){
//     // 处理新数据
//     while(rxbuffer -> old_pos != dma_write_ptr){
//       uint8_t ch = rxbuffer -> buf[rxbuffer -> old_pos];
//       //存字节
//       line_temp[line_pos++] = ch;
//       //利用通信协议，判断什么时候该发
//       if(ch == '\n'){
// 				Speed_PID(line_temp);
//         HAL_UART_Transmit(&huart3, line_temp, line_pos, 10);
//         line_pos = 0; // 重置行数据位置
//       }
//       //指针往后面挪，挪到末尾自动回到0
//       rxbuffer -> old_pos ++;
//       if(rxbuffer -> old_pos >= rxbuffer -> buf_size)  rxbuffer -> old_pos = 0; // 循环利用缓冲区
//       if(line_pos >= 60) line_pos = 0; // 防止行数据溢出
//     }
//   }
// }


