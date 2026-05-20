#include "imu.h"
#include "test.h"

IMU_Data cur_imu;
uint8_t rx_buf[64];
char yaw_str[32];

RXBUFFER_U Rx_Buffer = {
	.buf = rx_buf,
	.buf_size = sizeof(rx_buf),
	.old_pos = 0,
    .line_pos = 0,
    .line_temp = {0},
};

//！IMU的数据接收可以写成队列，此处需要修改，注意防接错或者丢包
//！接收数据可以和编码器算出的数据做一下整合，做个置信区间

//在串口中断或DMA处理函数中调用
void IMU_Update(uint8_t *data){
    //组合16位数据并除以100得到浮点角度
    if(data[0] == 0x55){
        //校验和验证
        uint8_t sum = 0;
        for(int i = 0; i < 10; i++) sum += data[i];
        if(sum != data[10]) return; //校验失败，丢弃数据  
        //加速度
        if(data[1] == 0x51){
            float ax = (short)(data[3] << 8 | data[2]) / 32768.0f * 16.0f; //横滚
            float ay = (short)(data[5] << 8 | data[4]) / 32768.0f * 16.0f; //俯仰
            float az = (short)(data[7] << 8 | data[6]) / 32768.0f * 16.0f; //偏航
            cur_imu.ax = ax;
            cur_imu.ay = ay;
            cur_imu.az = az;
        }
        //角速度
        else if(data[1] == 0x52){
           float wx = (short)(data[3] << 8 | data[2]) / 32768.0f * 2000.0f; //横滚
           float wy = (short)(data[5] << 8 | data[4]) / 32768.0f * 2000.0f; //俯仰
           float wz = (short)(data[7] << 8 | data[6]) / 32768.0f * 2000.0f; //偏航
           cur_imu.wx = wx;
           cur_imu.wy = wy;
           cur_imu.wz = wz;
        }
        //角度
        else if(data[1] == 0x53){
           float roll = (short)(data[3] << 8 | data[2]) / 32768.0f * 180.0f; //横滚
           float pitch = (short)(data[5] << 8 | data[4]) / 32768.0f * 180.0f; //俯仰
           float yaw = (short)(data[7] << 8 | data[6]) / 32768.0f * 180.0f; //偏航
           cur_imu.roll = roll;
           cur_imu.pitch = pitch;
           cur_imu.yaw = yaw;
        }
        else return; //非角度数据，丢弃
    } 
}


void Process_IMU_Data(RXBUFFER_U *rxbuffer){
  //获取DMA还需搬运字节数
  uint16_t remaining_space = __HAL_DMA_GET_COUNTER(huart4.hdmarx);
  //计算DMA搬运到的数组下标,取余避免越界
  uint8_t dma_write_ptr = (rxbuffer -> buf_size - remaining_space) % rxbuffer -> buf_size;
  //写指针变化，说明有新数据到达
  if(dma_write_ptr != rxbuffer -> old_pos){
    // 处理新数据
    while(rxbuffer -> old_pos != dma_write_ptr){
      uint8_t ch = rxbuffer -> buf[rxbuffer -> old_pos];
      //存字节
      rxbuffer -> line_temp[rxbuffer -> line_pos++] = ch;
      //第一位不是0x55直接舍弃
       if(rxbuffer -> line_pos == 1 && ch != 0x55){
        rxbuffer -> line_pos = 0; // 重置行数据位置
        goto next;
       }
        if(rxbuffer -> line_pos == 11){
           uint8_t sum = 0;
           for(int i = 0; i < 10; i++) sum += rxbuffer -> line_temp[i];
           if(sum != rxbuffer -> line_temp[10]){
             rxbuffer -> line_pos = 0; // 重置行数据位置
             goto next;
           } 
        //利用通信协议，判断什么时候该发
        //加速度
            if(rxbuffer -> line_temp[1] == 0x51){
                float ax = (short)(rxbuffer -> line_temp[3] << 8 | rxbuffer -> line_temp[2]) / 32768.0f * 16.0f; //横滚
                float ay = (short)(rxbuffer -> line_temp[5] << 8 | rxbuffer -> line_temp[4]) / 32768.0f * 16.0f; //俯仰
                float az = (short)(rxbuffer -> line_temp[7] << 8 | rxbuffer -> line_temp[6]) / 32768.0f * 16.0f; //偏航
                cur_imu.ax = ax;
                cur_imu.ay = ay;
                cur_imu.az = az;
            }
            //角速度
            else if(rxbuffer -> line_temp[1] == 0x52){
                float wx = (short)(rxbuffer -> line_temp[3] << 8 | rxbuffer -> line_temp[2]) / 32768.0f * 2000.0f; //横滚
                float wy = (short)(rxbuffer -> line_temp[5] << 8 | rxbuffer -> line_temp[4]) / 32768.0f * 2000.0f; //俯仰
                float wz = (short)(rxbuffer -> line_temp[7] << 8 | rxbuffer -> line_temp[6]) / 32768.0f * 2000.0f; //偏航
                cur_imu.wx = wx;
                cur_imu.wy = wy;
                cur_imu.wz = wz;
            }
            //角度
            else if(rxbuffer -> line_temp[1] == 0x53){
                float roll = (short)(rxbuffer -> line_temp[3] << 8 | rxbuffer -> line_temp[2]) / 32768.0f * 180.0f; //横滚
                float pitch = (short)(rxbuffer -> line_temp[5] << 8 | rxbuffer -> line_temp[4]) / 32768.0f * 180.0f; //俯仰
                float yaw = (short)(rxbuffer -> line_temp[7] << 8 | rxbuffer -> line_temp[6]) / 32768.0f * 180.0f; //偏航
                cur_imu.roll = roll;
                cur_imu.pitch = pitch;
                cur_imu.yaw = yaw;
                
            }
            //sprintf(yaw_str, "Yaw: %.2f \n", cur_imu.wz);
            //HAL_UART_Transmit(&huart3, (uint8_t*)yaw_str, strlen(yaw_str), 100);
            rxbuffer -> line_pos = 0; // 重置行数据位置
        }
    
        //指针往后面挪，挪到末尾自动回到0
        next:
        rxbuffer -> old_pos ++;
        if(rxbuffer -> old_pos >= rxbuffer -> buf_size)  rxbuffer -> old_pos = 0; // 循环利用缓冲区
        if(rxbuffer -> line_pos >= 60) rxbuffer -> line_pos = 0; // 防止行数据溢出
    } 
  }
}