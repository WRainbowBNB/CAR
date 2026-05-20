#include "PS2.h"


uint16_t Handkey;
uint8_t Data[9] = {0x00}; // 存储手柄回传的所有数据
uint8_t last_PS2_time = 0;
volatile uint8_t last_PS2_flag = 0; //检测数据是否有效标志
uint8_t check[4] = {0x00};
uint8_t last_PS2_cnt = 0;

// 私有延时函数 (static 修饰，避免外部冲突)
static void delay_us(uint32_t us) {
    volatile uint32_t i; 
    // F407 168MHz，这里的系数需要实测，或者使用更精确的DWT计数
    // 粗略估算：168Mhz / 5 cycles per loop approx
    i = us * (SystemCoreClock / 1000000); 
    while(i--);
}

// 核心通信函数：发送1字节同时接收1字节
// PS2协议：LSB First (低位先出), 在时钟下降沿采样/变化
static uint8_t PS2_TxRxByte(uint8_t byte)
{
    uint8_t i, ref = 0x00;
    
    for (i = 0; i < 8; i++)
    {
        // 1. 准备命令数据 (发送给手柄)
        if (byte & 0x01) PS2_CMD_H();
        else             PS2_CMD_L();
        
        // 2. 拉低时钟，告诉手柄"我要读写了"
        PS2_CLK_L(); 
        delay_us(16); // 等待信号稳定
        
        // 3. 读取手柄发回的数据 (手柄在CLK下降沿放置数据)
        if (PS2_DAT_READ()) ref |= (1 << i);
        
        // 4. 拉高时钟，结束这一位的传输
        PS2_CLK_H(); 
        delay_us(16);
        
        // 移位，准备下一位
        byte >>= 1;
    }
    return ref;
}

// cmd: 命令数组, len: 长度
static void PS2_Cmd(uint8_t* cmd, uint8_t len)
{
    PS2_CS_L();
    delay_us(16);
    for(uint8_t i = 0; i < len; i++)
    {
        PS2_TxRxByte(cmd[i]);
    }
    PS2_CS_H();
    
    delay_us(16);
}

// --- 强制设置红灯模式函数 ---
void PS2_SetInit(void)
{
    // 1. 进入配置模式命令
    uint8_t enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x00};
    // 2. 开启红灯(模拟)模式并锁定命令 (0x01=Analog, 0x03=Locked)
    uint8_t turn_on_analog[] = {0x01, 0x44, 0x00, 0x01, 0x03};
    // 3. 配置震动电机命令 (可选，不配置可能导致大电机不转)
    uint8_t config_motor[] = {0x01, 0x4D, 0x00, 0x00, 0x01};
    // 4. 保存并退出配置模式命令
    uint8_t exit_config[] = {0x01, 0x43, 0x00, 0x00, 0x5A};

    // --- 开始发送配置序列 ---
    
    // PS2手柄上电需要一点时间，先读几次让它醒过来
    PS2_ReadData();
    PS2_ReadData();

    // 步骤1: 进入配置模式
    PS2_Cmd(enter_config, 5);
    delay_us(1000); // 稍作延时确保指令被处理

    // 步骤2: 设为红灯模式 (0x73) 且 锁定 (防止按MODE键关闭)
    PS2_Cmd(turn_on_analog, 5);
    delay_us(1000);

    // 步骤3: 开启震动功能 (如果你需要震动的话)
    PS2_Cmd(config_motor, 5);
    delay_us(1000);

    // 步骤4: 退出配置，完成！
    PS2_Cmd(exit_config, 5);
    delay_us(1000);
}

// 初始化引脚电平
void PS2_Init(void)
{
    PS2_CS_H();
    PS2_CLK_H();
    PS2_CMD_H();
    PS2_SetInit(); //转到遥控模式
}


// 读取手柄所有数据
void PS2_ReadData(void)
{
    PS2_CS_L(); // 片选拉低，开始
    delay_us(16);

    // 发送标准握手序列
    Data[0] = PS2_TxRxByte(0x01); // Start
    Data[1] = PS2_TxRxByte(0x42); // Request
    Data[2] = PS2_TxRxByte(0x00); // Idle

    // 读取后续的模拟量和按键数据
    Data[3] = PS2_TxRxByte(0x00); // Buttons 1
    Data[4] = PS2_TxRxByte(0x00); // Buttons 2
    Data[5] = PS2_TxRxByte(0x00); // RX
    Data[6] = PS2_TxRxByte(0x00); // RY
    Data[7] = PS2_TxRxByte(0x00); // LX
    Data[8] = PS2_TxRxByte(0x00); // LY
    
    PS2_CS_H(); // 片选拉高，结束
    delay_us(16);

    last_PS2_time = HAL_GetTick();  //更新最后读取时间
}

//数据校验（防断联）
void Data_check(uint8_t *Data){
    if(Data[1] == 0x73 && Data[2] == 0x5A){
        last_PS2_flag = 1; //数据有效
    }else{
        last_PS2_flag = 0; //数据无效
    }
}

void Data_To_Frame(uint8_t *Data, uint8_t *frameBuf){
    frameBuf[0] = 0xAA;
    memcpy(&frameBuf[1], Data, 9);
    frameBuf[10] = 0xFF;
}

// 解析按键状态
uint16_t PS2_GetButtons(void)
{
    // 拼接两个字节的按键数据
    Handkey = (Data[3] | (Data[4] << 8));
    // PS2按键按下是0，未按下是1。为了符合直觉，取反返回 (按下=1)
    return ~Handkey; 
    
}

