#include "time_slice.h"

char message[50];

//****机械臂可以改成模拟量控制 */
void time_slice_Init(time_slice* poll, uint32_t interval, void(*callback)(void)){
    poll -> in_last_time = HAL_GetTick();
    poll -> in_interval = interval;
    poll -> in_callback = callback;
}

void time_slice_run(time_slice* poll){
    if(HAL_GetTick() - poll->in_last_time >= poll -> in_interval){
        poll -> in_last_time = HAL_GetTick();
        poll -> in_callback();
    }
}

//可能用到的回调
void PS2_collect_callback(void){
    // 1. 读取数据
    PS2_ReadData();
    Data_check(Data);
    //last_PS2_flag = 0;
    // 2. 解析数据
    uint16_t ButtonStatus = PS2_GetButtons();
    //防失联失控
    if(last_PS2_flag != 0){
        events = Map_Button_To_Event(ButtonStatus);
        State_Transmission();   //状态转换
    }   
        
}

void OLED_callback(void){
    OLED_NewFrame();
    //数据同步
    UI_Data.display_mode = (uint8_t)currentstates;
    //根据模式显示不同的信息
    switch(UI_Data.display_mode){
        case STATE_REMOTE:
            OLED_PrintString(0, 5, "遥控模式", &font16x16);
            break;
        case STATE_TRACE:
            OLED_PrintString(0, 5, "循迹模式", &font16x16);
            break;
        case STATE_IDLE:
            OLED_PrintString(0, 5, "空闲模式", &font16x16);
            break;
    }
    OLED_DrawLine(0, 23, 127, 23);//分割线
    // 显示速度
    UI_Data.display_speed = target_vy;

    if(last_PS2_flag == 0){
        OLED_PrintString(97, 25, "No Signal", &font12x12);
    }else{
        OLED_PrintString(0, 25, "speed: ", &font16x16);
        sprintf(message, "%.2f", UI_Data.display_speed);
        OLED_PrintString(50, 25, message, &font16x16);
        OLED_PrintString(97, 30, "cm/s", &font12x12);
    }
    // 显示速度条
    OLED_DrawRectangle(0, 50, 127, 10);
    // 最大速度 200，按比例填充
     float display_speed;
     display_speed = fabs(UI_Data.display_speed);
    uint8_t bar_len = (uint8_t)(display_speed / 200.0f * 125.0f);
    OLED_DrawFilledRectangle(1, 51, bar_len, 9);
    OLED_ShowFrame();
}
